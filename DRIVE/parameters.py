SEED = 0

# RLHF
REWARD_MODEL_N = 3 * 2
SEQ_LEN = 1 # 30
DATA_TYPE = 'metrics' # 'metrics'  'image'
DATASET_SAVE = 'reward_model_data/'

# VAE/AE
MODEL_TYPE = 'VAE'
Z_DIM = 64
CHECKPOINT = 129

# PPO
LR_ADAPTIVE = False
LEARNING_RATE_MIN = 1e-6
if LR_ADAPTIVE:
    DESIRED_KL = 0.01
    LEARNING_RATE = 1e-3
    LOSS_VALUE_NORM = True
else:
    DESIRED_KL = None
    LEARNING_RATE = 1e-4
    LOSS_VALUE_NORM = False
CLIP_NORM = False
LR_DECAY = True
if LR_ADAPTIVE:
    LR_DECAY = False

REWARD_SCALING = False
REWARD_CENTERING = False
REWARD_NORMALIZATION = False
if REWARD_SCALING:
    REWARD_CENTERING = False
    REWARD_NORMALIZATION = False
if REWARD_CENTERING:
    REWARD_NORMALIZATION = False

STATE_NORM = False
ADVANTAGE_NORM = True
LAYER_NORM = True
GRAD_CLIP = False
DISCOUNT_FACTOR = 0.99
GAE_LAMBDA = 0.95
PPO_EPSILON = 0.2
VALUE_SCALE = 1.0
ENTROPY_SCALE = 0.01
PPO_MODEL = "ppo/"
ACTION_STD_INIT = 1.0 # 0.1更平滑，1.0更抖动，先1.0试试

# Additional states
measurements_to_include = ["steer",
                           "steer_expect",
                           "throttle",
                           "speed",
                           # "pitch",
                           "waypoints",
                           ]

# Environment
TOWN = 'Town07' # Town02 或 Town07
ACTIVATE_RENDER = True
START_CARLA = False # False
SYNCHRONOUS = True
IF_ROS = True
RLHF = True
SAVE_REWARD_MODEL_DATA = False
FPS = 15
ACTION_SMOOTHING = [0.9, 0.6] # SB3设置ACTION_SMOOTHING = 0.75, 0.9 -> 1, [0.9, 0.6] -> 2

if TOWN == 'Town02':
    VAE_MODEL = "vae/models/vae_64"
else:
    VAE_MODEL = "vae/models/seg_bce_cnn_zdim64_beta1_kl_tolerance0.0_data/" # tensorflow -> seg_bce_cnn_zdim64_beta1_kl_tolerance0.0_data  torch -> my_model_vae

# Training
NETWORK_UPDATE_EPOCHS = 10 # 4
BATCH_SIZE = 2048 # 256
MINIBATCH_SIZE = 64 # 64
EVAL_INTERVAL = 20
TOTAL_STEPS = 1e6 # 6e5
EPISODE_LENGTH = 2048 # 1024
BREAK_STEPS = 1e6
RECORD_TO_FILE = "videos/"

n_points = 15

# Reward
low_speed_timer = 0
max_distance    = 3.0  # Max distance from center before terminating
target_speed    = 12.0 # kmh # 15
min_speed = 10.0 # km/h # 最小速度
max_speed = 20.0 # km/h # 最大速度
max_std_center_lane = 0.4
max_angle_center_lane = 20