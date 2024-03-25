# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Loss functions
"""

import torch
if torch.__version__ >= '1.8':
    import torch_npu
import torch.nn as nn

from utils.metrics import bbox_iou
from utils.torch_utils import is_parallel


def smooth_BCE(eps=0.1):  # https://github.com/ultralytics/yolov3/issues/238#issuecomment-598028441
    # return positive, negative label smoothing BCE targets
    return 1.0 - 0.5 * eps, 0.5 * eps


class BCEBlurWithLogitsLoss(nn.Module):
    # BCEwithLogitLoss() with reduced missing label effects.
    def __init__(self, alpha=0.05):
        super(BCEBlurWithLogitsLoss, self).__init__()
        self.loss_fcn = nn.BCEWithLogitsLoss(reduction='none')  # must be nn.BCEWithLogitsLoss()
        self.alpha = alpha

    def forward(self, pred, true):
        loss = self.loss_fcn(pred, true)
        pred = torch.sigmoid(pred)  # prob from logits
        dx = pred - true  # reduce only missing label effects
        # dx = (pred - true).abs()  # reduce missing label and false label effects
        alpha_factor = 1 - torch.exp((dx - 1) / (self.alpha + 1e-4))
        loss *= alpha_factor
        return loss.mean()


class FocalLoss(nn.Module):
    # Wraps focal loss around existing loss_fcn(), i.e. criteria = FocalLoss(nn.BCEWithLogitsLoss(), gamma=1.5)
    def __init__(self, loss_fcn, gamma=1.5, alpha=0.25):
        super(FocalLoss, self).__init__()
        self.loss_fcn = loss_fcn  # must be nn.BCEWithLogitsLoss()
        self.gamma = gamma
        self.alpha = alpha
        self.reduction = loss_fcn.reduction
        self.loss_fcn.reduction = 'none'  # required to apply FL to each element

    def forward(self, pred, true):
        loss = self.loss_fcn(pred, true)
        # p_t = torch.exp(-loss)
        # loss *= self.alpha * (1.000001 - p_t) ** self.gamma  # non-zero power for gradient stability

        # TF implementation https://github.com/tensorflow/addons/blob/v0.7.1/tensorflow_addons/losses/focal_loss.py
        pred_prob = torch.sigmoid(pred)  # prob from logits
        p_t = true * pred_prob + (1 - true) * (1 - pred_prob)
        alpha_factor = true * self.alpha + (1 - true) * (1 - self.alpha)
        modulating_factor = (1.0 - p_t) ** self.gamma
        loss *= alpha_factor * modulating_factor

        if self.reduction == 'mean':
            return loss.mean()
        elif self.reduction == 'sum':
            return loss.sum()
        else:  # 'none'
            return loss


class QFocalLoss(nn.Module):
    # Wraps Quality focal loss around existing loss_fcn(), i.e. criteria = FocalLoss(nn.BCEWithLogitsLoss(), gamma=1.5)
    def __init__(self, loss_fcn, gamma=1.5, alpha=0.25):
        super(QFocalLoss, self).__init__()
        self.loss_fcn = loss_fcn  # must be nn.BCEWithLogitsLoss()
        self.gamma = gamma
        self.alpha = alpha
        self.reduction = loss_fcn.reduction
        self.loss_fcn.reduction = 'none'  # required to apply FL to each element

    def forward(self, pred, true):
        loss = self.loss_fcn(pred, true)

        pred_prob = torch.sigmoid(pred)  # prob from logits
        alpha_factor = true * self.alpha + (1 - true) * (1 - self.alpha)
        modulating_factor = torch.abs(true - pred_prob) ** self.gamma
        loss *= alpha_factor * modulating_factor

        if self.reduction == 'mean':
            return loss.mean()
        elif self.reduction == 'sum':
            return loss.sum()
        else:  # 'none'
            return loss

class DeterministicIndex(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x, indices_list):
        ctx.x = x
        ctx.indices_list = indices_list
        # return x[indices_list[0], indices_list[1], :, indices_list[2]]
        return x[indices_list[0].long(), indices_list[1].long(), :, indices_list[2].long()]
    
    # def forward(ctx, x, indices_list):
    #     ctx.x = x
    #     ctx.indices_list = indices_list
    #     return x[indices_list[0], indices_list[1], :, indices_list[2]]

    @staticmethod
    def backward(ctx, grad_output):
        tmp = torch.zeros_like(ctx.x)
        ind0, ind1, ind2 = ctx.indices_list
        ind0=ind0.long()
        ind1=ind1.long()
        ind2=ind2.long()
        tmp[ind0, ind1, :, ind2] = grad_output
        
        return tmp, None

class ComputeLoss:
    # Compute losses
    def __init__(self, model, autobalance=False):
        self.sort_obj_iou = False
        device = next(model.parameters()).device  # get model device
        h = model.hyp  # hyperparameters

        # Define criteria
        BCEcls = nn.BCEWithLogitsLoss(pos_weight=torch.tensor([h['cls_pw']]), reduction='none').to(device)
        BCEobj = nn.BCEWithLogitsLoss(pos_weight=torch.tensor([h['obj_pw']]), reduction='none').to(device)

        # Class label smoothing https://arxiv.org/pdf/1902.04103.pdf eqn 3
        self.cp, self.cn = smooth_BCE(eps=h.get('label_smoothing', 0.0))  # positive, negative BCE targets

        # Focal loss
        g = h['fl_gamma']  # focal loss gamma
        if g > 0:
            BCEcls, BCEobj = FocalLoss(BCEcls, g), FocalLoss(BCEobj, g)

        det = model.module.model[-1] if is_parallel(model) else model.model[-1]  # Detect() module
        self.balance = {3: [4.0, 1.0, 0.4]}.get(det.nl, [4.0, 1.0, 0.25, 0.06, .02])  # P3-P7
        self.ssi = list(det.stride).index(16) if autobalance else 0  # stride 16 index
        self.BCEcls, self.BCEobj, self.gr, self.hyp, self.autobalance = BCEcls, BCEobj, 1.0, h, autobalance
        self.index1, self.range_nb = None, dict()
        for k in 'na', 'nc', 'nl', 'anchors':
            setattr(self, k, getattr(det, k))
    
    def get_index(self, device):
        if self.index1 is None:
            self.index1 = torch.tensor([0, 1, 2, 3, 4, 5], device=device)

    def get_range_nb(self, n, device):
        if n not in self.range_nb:
            self.range_nb[n] = torch.arange(n, device=device).long()
    
    def __call__(self, p, targets):  # predictions, targets, model
        device = targets.device
        lcls, lbox, lobj = torch.zeros(1, device=device), torch.zeros(1, device=device), torch.zeros(1, device=device)
        tcls, tbox, indices, anchors, targets_mask, targets_sum_mask = self.build_targets(p, targets)  # targets

        b_cat, a_cat, g_cat = [], [], []
        p_cat = torch.cat([pi.flatten(3) for pi in p], 3)
        offset = 0
        
        # for i, pi in enumerate(p):
        #     _, _, _, h, w = pi.shape
        #     b, a, gj, gi = indices[i]
        #     b_cat.append(b)
        #     a_cat.append(a)
        #     g_cat.append(gj * w + gi + offset)
        #     offset += h * w
        
        for i, pi in enumerate(p):
            _, _, _, h, w = pi.shape
            b, a, gj, gi = indices[i]
            b_cat.append(b)
            a_cat.append(a)
            t = gj * w
            t = torch.tensor(t, dtype=torch.float32) + torch.tensor(gi, dtype=torch.float32) + torch.tensor(offset, dtype=torch.float32)
            # t = t + torch.tensor(t, dtype=torch.float16) + torch.tensor(gi, dtype=torch.float16) + torch.tensor(offset, dtype=torch.float16)
            # t = t + torch.tensor(t, dtype=torch.int32) + torch.tensor(gi, dtype=torch.int32) + torch.tensor(offset, dtype=torch.int32)
            g_cat.append(t)
            offset = offset + h * w
            
        b_cat = torch.cat(b_cat)
        a_cat = torch.cat(a_cat)
        g_cat = torch.cat(g_cat)
        tcls_cat = torch.cat(tcls)
        tbox_cat = torch.cat(tbox, 1)
        anchors_cat = torch.cat(anchors, 0)
        all_mask_cat = torch.cat(targets_mask, 1)
        tobj = torch.zeros_like(p_cat[:, :, 0, :])                      # target obj
        sum_mask_cat = torch.stack(targets_sum_mask)
        sum_mask = torch.sum(sum_mask_cat)      
        n = b_cat.shape[0]
        bs = p_cat.shape[0]
        self.get_range_nb(n, device)
        if sum_mask.item() > 0:
            ps = DeterministicIndex.apply(p_cat, (b_cat, a_cat, g_cat)).permute(1, 0).contiguous()
            pxy = ps[:2]
            pwh = ps[2:4]
            pxy = pxy.sigmoid() * 2. - 0.5
            pwh = (pwh.sigmoid() * 2) ** 2 * anchors_cat.T
            pbox = torch.cat((pxy, pwh), 0)  # predicted box
            iou = torch_npu.npu_ciou(pbox, tbox_cat, is_cross=False, trans=True)  # iou(prediction, target)
            iou = iou * all_mask_cat + (1. - all_mask_cat)
            valid_mask = sum_mask_cat > 0
            lbox += ((1.0 - iou).reshape(self.nl, -1).sum(1)[valid_mask] / sum_mask_cat[valid_mask]).sum()  # iou loss

            # Objectness
            # iou = iou * all_mask_cat
            # score_iou = iou.detach().clamp(0).type(tobj.dtype)
            # if self.sort_obj_iou:
            #     sort_id = torch.argsort(score_iou)
            #     b_cat, a_cat, g_cat, score_iou = b_cat[sort_id], a_cat[sort_id], g_cat[sort_id], score_iou[sort_id]
            # tobj[b_cat, a_cat, g_cat] = (1.0 - self.gr) + self.gr * score_iou  # iou ratio
            
            # Objectness
            iou = iou * all_mask_cat
            score_iou = iou.detach().clamp(0).type(tobj.dtype)
            if self.sort_obj_iou:
                sort_id = torch.argsort(score_iou)
                b_cat, a_cat, g_cat, score_iou = b_cat[sort_id].long(), a_cat[sort_id].long(), g_cat[sort_id].long(), score_iou[sort_id]
            tobj[b_cat.long(), a_cat.long(), g_cat.long()] = (1.0 - self.gr) + self.gr * score_iou  # iou ratio

            # Classification
            if self.nc > 1:  # cls loss (only if multiple classes)
                tmp = ps[5:, :]
                tmp = tmp * all_mask_cat - (1. - all_mask_cat) * 50.
                t = torch.full_like(tmp, 0)  # targets
                t[tcls_cat, self.range_nb[n]] = 1 
                t *= all_mask_cat
                lcls += (self.BCEcls(tmp, t).reshape(t.shape[0], self.nl, -1).sum(-1).sum(0)[valid_mask]
                        / (sum_mask_cat * t.shape[0])[valid_mask].float()).sum()
        obj_all = self.BCEobj(p_cat[:, :, 4, :], tobj).sum(0).sum(0)
        offset = 0
        for i, pi in enumerate(p):
            _, _, _, h, w = pi.shape
            range_p = h * w
            obji = obj_all[offset:offset+range_p].sum() / (range_p * self.nl * bs)
            lobj += obji * self.balance[i]  # obj loss
            offset += range_p
            if self.autobalance:
                self.balance[i] = self.balance[i] * 0.9999 + 0.0001 / obji.detach().item()

        if self.autobalance:
            self.balance = [x / self.balance[self.ssi] for x in self.balance]
        lbox *= self.hyp['box']
        lobj *= self.hyp['obj']
        lcls *= self.hyp['cls']

        total_loss = lbox + lobj + lcls
        return total_loss * bs, torch.cat((lbox, lobj, lcls, total_loss)).detach()

    def build_targets(self, p, targets):
        # Build targets for compute_loss(), input targets(image,class,x,y,w,h)
        self.get_index(targets.device)
        na, nt = self.na, targets.shape[1]  # number of anchors, targets

        batch_size = p[0].shape[0]
        nt_max = 32 * batch_size
        while nt > nt_max:
            nt_max *= 2
            print('**************** nt max=', nt_max)
        max_target = torch.zeros(6, nt_max, device=targets.device)   #  (6, 1024)
        max_target[0, :nt] = targets[0, :]
        max_target[1, :nt] = targets[1, :]
        max_target[2, :nt] = targets[2, :]
        max_target[3, :nt] = targets[3, :]
        max_target[4, :nt] = targets[4, :]
        max_target[5, :nt] = targets[5, :]

        tcls, tbox, indices, anch, targets_mask, targets_sum_mask = [], [], [], [], [], []
        gain = torch.ones(6, device=targets.device)  # normalized to gridspace gain
        off_list = [
            torch.tensor([[1.], [0.]], device=targets.device),
            torch.tensor([[0.], [1.]], device=targets.device),
            torch.tensor([[-1.], [0.]], device=targets.device),
            torch.tensor([[0.], [-1.]], device=targets.device),
        ]
        at = torch.arange(na).view(na, 1).repeat(1, nt_max).to(targets.device)
        a = at.view(-1)
        a = torch.cat((a, a, a, a, a), 0)

        g = 0.5  # bias

        for i in range(self.nl):
            anchors = self.anchors[i].float()
            gain[2:] = torch.tensor(p[i].shape)[[4, 3, 4, 3]].float()  # xyxy gain

            # Match targets to anchors
            t, offsets = max_target * gain[:, None], 0
            all_mask = torch.zeros((15 * nt_max)).to(targets.device)
            sum_mask = torch.zeros((1)).to(targets.device)

            if nt:
                # Matches
                r = t[None, 4:6, :] / anchors[..., None]  # wh ratio
                fmask = torch.max(r, 1. / r).max(1)[0] < self.hyp['anchor_t'] # compare
                fmask = fmask.view(1, -1)
                # j = wh_iou(anchors, t[:, 4:6]) > model.hyp['iou_t']  # iou(3,n)=wh_iou(anchors(3,2), gwh(n,2))
                t = t.repeat(1, 1, na).view(6, -1)  # filter

                # Offsets
                gxy = t[2:4]
                z = torch.zeros_like(gxy)

                jk = (gxy % 1. < g) & (gxy > 1.)
                lm = (gxy % 1. > (1. - g)) & (gxy < (gain[[2, 3]][:, None] - 1.))
                jk, lm = jk & fmask, lm & fmask
                all_mask = torch.cat((fmask, jk, lm), 0).view(1, -1).float()
                t = torch.cat((t, t, t, t, t), 1)
                offsets = torch.cat((z, z + off_list[0], z + off_list[1], z + off_list[2], z + off_list[3]), 1) * g
                sum_mask = all_mask.sum()
                t = t * all_mask

            # Define
            b = t[0].long().view(-1)   #(3072 * 5)
            c = t[1].long().view(-1)   #(3072 * 5)
            gxy = t[2:4] #(2, 3072 * 5)
            gwh = t[4:6] #(2, 3072 * 5)
            gij = gxy - offsets
            gij2 = gij.long()
            gi = gij2[0].view(-1) #(2, 3072 * 5)
            gj = gij2[1].view(-1) #(2, 3072 * 5)

            # Append
            indices.append((b, a, gj, gi))  # image, anchor, grid indices
            tbox.append(torch.cat((gxy - gij2.float(), gwh), 0))  # box
            anch.append(anchors[a])  # anchors
            tcls.append(c)  # class
            targets_mask.append(all_mask)
            targets_sum_mask.append(sum_mask)

        return tcls, tbox, indices, anch, targets_mask, targets_sum_mask
