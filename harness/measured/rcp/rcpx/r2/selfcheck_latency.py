# SPDX-License-Identifier: AGPL-3.0-only
"""Measure camera latency by matching an in-motion frame to joint readings."""
import math
import traceback
import numpy as np
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import selfcheck_fit as fit
from rcpx.r2 import selfcheck_image as image
GN_STEPS = 2  # one-dimensional fit iterations


def observer(ex, joints, q, floor_z):
    record = {"joint_index": joints[0], "gain": 0.0, "taken": False}
    try:
        gains = fit.picture_gain(ex.model, q, floor_z)
        j = max(joints, key=lambda k: gains[k])
        record.update(joint_index=j, gain=float(gains[j]))
    except Exception as error:
        record["why"] = f"{type(error).__name__}: {error}"
        ex._journal("r2_problem", package="selfcheck", where="camera latency", trace=traceback.format_exc())
    j = record["joint_index"]
    def capture(joint, side, plan, k, before, after, out):
        if record["taken"] or joint != j:
            return
        points = [plan._sc_q0] + plan.path
        steps = [abs(y[j] - x[j]) for x, y in zip(points, points[1:])]
        if k != int(np.argmax(steps)):
            return
        try:
            record.update(taken=True, frame=np.asarray(ex.backend.render("wrist")),
                          q=after.q.copy(), q_before=before.q.copy(), qd=float(after.qd[j]))
        except Exception as error:
            record.update(taken=True, why=f"{type(error).__name__}: {error}")
            ex._journal("r2_problem", package="selfcheck", where="camera latency", trace=traceback.format_exc())
    return record, capture


def measure(ex, wiggle, record, table, floor_z):
    name = ex.model.joint_names[record["joint_index"]]
    out = {"latency_s": None, "resolution_s": None, "joint": name, "known": False}
    try:
        j, gain = record["joint_index"], record["gain"]
        if record.get("why"):
            out["why"] = record["why"]
            return out
        if not record["taken"]:
            out["why"] = "no in-motion frame of this joint was taken"
            return out
        qd = record["qd"]
        if qd == 0 or gain <= 0:
            out["why"] = "the selected joint had no resolved picture speed"
            return out
        body = fit.Body(ex.model, wiggle["q_ref"], floor_z)
        reference = image.Reference(wiggle["ref_img"], image.robot_mask(ex.model, wiggle["q_ref"], floor_z, ex.cfg["mount_half_m"]))
        th = body.theta0()
        matched = image.measure(reference, record["frame"], [(1, body.homography(record["q"], th))])
        if not matched["ok"]:
            out["why"] = matched["why"]
            return out
        grid = fit._grid(ex.model)
        seen = image._warp(matched["H"], grid)[0]
        x = float(record["q"][j] - wiggle["q_ref"][j])
        def predicted(turn):
            q = record["q"].copy()
            q[j] = wiggle["q_ref"][j] + turn
            return image._warp(body.homography(q, th), grid)[0]
        for _ in range(GN_STEPS):
            p = predicted(x)
            derivative = (predicted(x + fit.DEG) - predicted(x - fit.DEG)) / (fit.DEG + fit.DEG)
            den = float(np.sum(derivative ** 2))
            if den == 0:
                out["why"] = "the picture fit has no joint direction"
                return out
            x += float(np.sum(derivative * (seen - p))) / den
        latency = (record["q"][j] - (wiggle["q_ref"][j] + x)) / qd
        resolution = fit.ROBUST_PX / (gain * abs(qd))
        out.update(latency_s=latency, resolution_s=resolution)
        row = table.get(name, {})
        if row.get("sign_ok") is not True or row.get("problem"):
            out["why"] = "this check did not find the joint turns the right way with unit gain"
            return out
        if abs(latency) <= resolution:
            value = 0.0
            out["why"] = "within resolution"
        elif latency < 0:
            out["why"] = "the measured frame leads its reading beyond the fit resolution"
            return out
        else:
            value = latency
        out.update(latency_s=value, known=True)
        bounds.of(ex).know.set("camera_latency_s", value, "self_test")
        ex.state["selfcheck"]["latency"] = dict(out)
    except Exception as error:
        out.update(known=False, why=f"{type(error).__name__}: {error}")
        ex._journal("r2_problem", package="selfcheck", where="camera latency", trace=traceback.format_exc())
    return out
