# Images in Dataset
N_IMAGES = 14910

# Ratio of dataset to use
TRAIN_SIZE = 0.7
VALID_SIZE = 0.2
TEST_SIZE  = 0.1

# base image directory + prefix
DATA_DIR = '/mnt/hard_data/carpose/dataset/images/car37_longtrim_'

# file creation directory
DEST_DIR = '/home/ugrads/WRK/carpose/src/yolov5/data/'

train_file = open(DEST_DIR + 'train.txt', 'w')
for idx in range(int(N_IMAGES * TRAIN_SIZE)):
    path = DATA_DIR + str(idx).zfill(6) + '.jpg'
    train_file.write(path + '\n')
train_file.close()

valid_file = open(DEST_DIR + 'valid.txt', 'w')
for idx in range(int(N_IMAGES * TRAIN_SIZE), int(N_IMAGES * (TRAIN_SIZE + VALID_SIZE))):
    path = DATA_DIR + str(idx).zfill(6) + '.jpg'
    valid_file.write(path + '\n')
valid_file.close()

test_file = open(DEST_DIR + 'test.txt', 'w')
for idx in range(int(N_IMAGES * (TRAIN_SIZE + VALID_SIZE)), N_IMAGES):
    path = DATA_DIR + str(idx).zfill(6) + '.jpg'
    test_file.write(path + '\n')
test_file.close()
