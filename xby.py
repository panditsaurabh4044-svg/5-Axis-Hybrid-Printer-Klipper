# /home/saurabh/klipper/klippy/kinematics/xby.py
#
# XBY kinematics for the modified Core-R-Theta style machine:
#   internal X = linear X from XB1/XB2 differential pair
#   internal Y = B-axis belt displacement in mm
#   internal Z = physical bed-Y linear axis
#
# Based on Klipper corexy.py, but with a custom native B-axis home path.
#
# Key custom behavior:
# - axis 0 (internal X) homes like CoreXY X
# - axis 1 (internal Y / B-axis belt mm) may home using either stepper_x
#   or stepper_y DIAG, selected by [printer] b_home_endstop_axis: x|y
# - axis 2 (internal Z / physical bed-Y) homes like cartesian Z
# - internal Y/B remains range-limited from config so front-end jogging
#   does not break, but native homing is still supported

import stepper

class XBYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']

        # Differential XB pair exactly like CoreXY
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)

        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)

        # Which DIAG/endstop rail should be used when homing internal Y/B
        # "x" means use stepper_x DIAG during G28.1 Y
        # "y" means use stepper_y DIAG during G28.1 Y
        bsrc = config.get('b_home_endstop_axis', default='x').strip().lower()
        self.b_home_endstop_axis = 0 if bsrc == 'x' else 1

        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord([r[0] for r in ranges])
        self.axes_max = toolhead.Coord([r[1] for r in ranges])

        # Keep B range valid immediately so UI/dashboard motion is stable.
        # X and physical bed-Y still require homing first.
        self.limits = [(1.0, -1.0), ranges[1], (1.0, -1.0)]

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        xb1 = pos[0]
        xb2 = pos[1]
        y_phys = pos[2]
        x_pos = 0.5 * (xb1 + xb2)
        b_mm = 0.5 * (xb1 - xb2)
        return [x_pos, b_mm, y_phys]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if "xyz"[i] in homing_axes:
                self.limits[i] = rail.get_range()

    def clear_homing_state(self, clear_axes):
        b_range = self.rails[1].get_range()
        for axis, axis_name in enumerate("xyz"):
            if axis_name in clear_axes:
                if axis == 1:
                    self.limits[axis] = b_range
                else:
                    self.limits[axis] = (1.0, -1.0)
        # Preserve B range for jog / macro friendliness.
        self.limits[1] = b_range

    def _home_axis_standard(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)

    def _home_b_axis(self, homing_state):
        # Native home of internal Y/B, but allow the DIAG source rail
        # to be selected independently of the axis coordinate rail.
        axis = 1
        axis_rail = self.rails[1]
        endstop_rail = self.rails[self.b_home_endstop_axis]

        position_min, position_max = axis_rail.get_range()
        hi = axis_rail.get_homing_info()

        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)

        homing_state.home_rails([endstop_rail], forcepos, homepos)

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if axis == 1:
                self._home_b_axis(homing_state)
            else:
                self._home_axis_standard(homing_state, axis, self.rails[axis])

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                    and (end_pos[i] < self.limits[i][0]
                         or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, bpos, zpos = move.end_pos[:3]
        if (xpos < limits[0][0] or xpos > limits[0][1]
                or bpos < limits[1][0] or bpos > limits[1][1]
                or zpos < limits[2][0] or zpos > limits[2][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            return
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return XBYKinematics(toolhead, config)
