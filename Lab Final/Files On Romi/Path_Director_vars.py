## @file Path_Director_vars.py
#  Shared configuration variables for the Path Director state machine.
#  Holds reference speeds and parameter slots for segment functions. 
#  Reduces the size of imports between the Path_Director and User_Input tasks.
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3


## Container for PathDirector adjustable variables.
class PD_vars:
    v_ref_DEFAULT: float = 200  # mm/s reference speed
    v_ref: float = 200  # mm/s reference speed
    next_state: int = 0  # next state
    var_1: float = 0  # next state input
    var_2: float = 0
    var_3: float = 0
    var_4: float = 0
