# SPDX-License-Identifier: AGPL-3.0-only
"""Generic mobile tool descriptions, sentences and practices."""

from rcpx import surfaces as S

PRACTICES = ("drive moves the base on the floor; torso raises or lowers the arm, its cameras and the robot frame.\n"
             "Points already located move with the robot's odometry (robot://world) and stay usable after a move.\n"
             "A mark takes a look made after the last move (0 control periods): looks from before a move are refused.\n"
             "home or park brings the arm in over the base, the pose a drive is made from.")

_RECORDS = (" Afterwards every point robot://world holds is re-expressed in the robot frame by the base's odometry "
            "(its data.odometry says how far it was carried and its estimated error), and looks taken before the "
            "move can no longer be marked or reused.")
_STOPS = (" It stops early when the hand feels a push, when the motion stalls (something holds it back), or when "
          "no control steps are left; the result's stopped says which (done when it arrived).")

def number(text):
    return {'type':'number','description':text}


def clear_schema():
    return {'oneOf':[{'type':'boolean'},{'type':'object','properties':{'look':{'type':'string'}},'required':['look'],'additionalProperties':False}],
            'description': "true after your own look when you judge the way clear, or {look: id}: recorded in the result; it changes no speed"}


TOOLS = [
    S._tool('drive','Drive the base ahead, sideways and turning together at its own rated speed, cut near stored points. The full arm and footprint are checked along the path.'+_STOPS+_RECORDS,
            S._obj({'forward_mm':number('travel ahead in mm; negative goes back; omitted means no forward travel'),
                    'left_mm':number('travel left in mm; negative goes right; omitted means no sideways travel'),
                    'turn_deg':number('turn left in degrees; negative goes right; one full turn at most'),
                    'clearance_mm':number('more clearance from stored points in mm; derived when omitted'),
                    'clear':clear_schema()}),{},'state the travel you want'),
    S._tool('torso','Set the height joint to its own coordinate, clamped to its model range.'+_STOPS+_RECORDS,
            S._obj({'height_mm':number('height joint coordinate in mm'),'clear':clear_schema()},('height_mm',)),{},'state the height you want'),
    S._tool('base_pose','Read base odometry, height, controller ratings and footprint without moving.',
            S._obj({}),{},'read where the base is')]

_HOW = {"done": "arrived", "stalled": "stopped because it stalled (something held it back)",
        "contact": "stopped because the hand felt a push", "steps_spent": "stopped because no control steps are left"}


def _how(stopped):
    return _HOW.get(stopped, f"stopped ({stopped})")


def drive_said(stopped, moved, records=0):
    return (f"The base moved {moved['forward_mm']:.1f} mm forward and {moved['left_mm']:.1f} mm left and turned "
            f"{moved['turn_deg']:.2f} deg, {_how(stopped)}; {records} stored point(s) were carried with it.")


def torso_said(stopped, height, clamped, records=0):
    clamp = ", the asked height clamped to its range," if clamped else ""
    return (f"The torso is at {height:.1f} mm{clamp} {_how(stopped)}; {records} stored point(s) were carried "
            "with it.")
