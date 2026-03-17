"""
Central constants for all robot behaviour and field geometry.

Edit values HERE — every robot file imports from this single source.
"""

# ═════════════════════════════════════════════════════════════════
#  FIELD GEOMETRY (mm) — SSL small field 5000 × 3000
# ═════════════════════════════════════════════════════════════════

FIELD_LENGTH      = 5000
FIELD_WIDTH       = 3000
HALF_LEN          = FIELD_LENGTH / 2
HALF_WID          = FIELD_WIDTH / 2
GOAL_WIDTH        = 1000
GOAL_HW           = GOAL_WIDTH / 2
GOAL_DEPTH        = 180

PENALTY_DEPTH     = 500
PENALTY_WIDTH     = 1000
PENALTY_HW        = PENALTY_WIDTH / 2
CENTER_RADIUS     = 500
FIELD_MARGIN      = 300

DEFENSE_DEPTH     = 1200
DEFENSE_HALF_WIDTH = 1200

# ═════════════════════════════════════════════════════════════════
#  ROBOT PHYSICAL LIMITS
# ═════════════════════════════════════════════════════════════════

ROBOT_RADIUS      = 90       # mm

MAX_SPEED         = 1.0      # m/s — absolute hardware speed limit

MAX_W             = 1.0      # rad/s — angular velocity cap
TURN_GAIN         = 1.5      # proportional gain for angle → w

# Field speeds as fraction of MAX_SPEED (clamped automatically)
SPRINT_SPEED      = 0.73 * MAX_SPEED   # max repositioning
CRUISE_SPEED      = 0.60 * MAX_SPEED   # medium approach
CHARGE_SPEED      = 0.47 * MAX_SPEED   # close-range drive
DRIBBLE_SPEED     = 0.33 * MAX_SPEED   # precise ball control
ONETOUCH_SPEED    = 0.53 * MAX_SPEED   # one-touch redirect

# Goalie-specific speeds (fraction of MAX_SPEED)
SAVE_SPEED        = 0.83 * MAX_SPEED   # shot-save sprint
POSITION_SPEED    = 0.53 * MAX_SPEED   # angle narrowing
CLEAR_SPEED       = 0.47 * MAX_SPEED   # dead-ball clearance
RETREAT_SPEED     = 0.67 * MAX_SPEED   # return to goal
DISTRIBUTE_SPEED  = 0.40 * MAX_SPEED   # dribble to pass

# ═════════════════════════════════════════════════════════════════
#  DISTANCES (mm)
# ═════════════════════════════════════════════════════════════════

KICK_RANGE        = 175      # trigger kick distance
KICK_DIST         = 175      # alias used by goalie
BALL_NEAR         = 420      # "close to ball" threshold
BEHIND_DIST       = 300      # lineup distance behind ball
AVOID_RADIUS      = 400      # swing-around radius
MAX_ADVANCE       = PENALTY_DEPTH - 50  # goalie must stay inside penalty box

PRESSURE_DIST     = 500      # mm — opponent "under pressure" radius
PASS_CLEAR        = 400      # mm — pass lane clearance

# ═════════════════════════════════════════════════════════════════
#  ANGULAR
# ═════════════════════════════════════════════════════════════════

FACE_BALL_GAIN    = 1.5      # goalie proportional gain for facing ball
ONETOUCH_ANGLE    = 0.8      # max angle offset for one-touch redirect

# ═════════════════════════════════════════════════════════════════
#  THRESHOLDS
# ═════════════════════════════════════════════════════════════════

SHOT_SPEED        = 500      # mm/s — incoming shot detection
CLEAR_BALL_SPEED  = 450      # mm/s — clearable ball speed
CLEAR_BALL_DIST   = 1100     # mm — go clear if this close
DANGER_ZONE       = HALF_LEN # mm — ball in our half
ONETOUCH_MIN_SPEED = 300     # mm/s — min ball speed for one-touch
BALL_MOVING_THRESH = 150     # mm/s — ball considered moving

# ═════════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═════════════════════════════════════════════════════════════════

FRICTION          = 0.4      # friction deceleration factor per second
BALL_HISTORY_SIZE = 7        # frames of ball position history
INTERCEPT_MAX_T   = 1.0      # max seconds to predict ahead
INTERCEPT_STEPS   = 12       # number of prediction steps

# ═════════════════════════════════════════════════════════════════
#  TIMING
# ═════════════════════════════════════════════════════════════════

LOOP_RATE         = 0.016    # ~60 Hz main loop sleep
FRAME_INTERVAL    = 0.04     # ~25 Hz frame fetch interval
KICK_COOLDOWN     = 0.22     # seconds between kicks
