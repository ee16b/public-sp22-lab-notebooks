import csv
import numpy as np
import os


def read_csv(filename):
    return np.genfromtxt(filename, dtype=np.float, delimiter=",")


def train_test_split(data, split_ratio=0.7):
    """Splits data into training and test set according to the split_ratio.

    Arguments:
        data: dataset as a numpy array
        split_ratio: fraction of dataset to split as training data (must be between 0 and 1)

    Returns:
        Training Data (size = split_ratio * size of original dataset)
        Test Data (size = (1 - split_ratio) * size of original dataset)
    """
    np.random.shuffle(data)
    train_data, test_data = data[:int(split_ratio *
                                      len(data)), :], data[int(split_ratio *
                                                               len(data)):, :]

    return train_data, test_data


def format_constant_energia(name, constant):
    # <Insert smug remark about left-pad>
    if len(name) < 37:
        padding = " " * (38 - len(name) - len("#define "))
    else:
        padding = "\t"
    return "#define {}{}{}".format(name, padding, constant)


def format_array_energia(name, array, dtype="float"):
    contents = ", ".join(map(str, array))
    return "{} {}[{}] = {{{}}};".format(dtype, name, len(array), contents)


def create_study():
    """
    Create a study for parameter tuning.
    """
    import optuna

    study_folder = 'HPO'
    CWD = os.getcwd()
    folder_path = os.path.join(CWD, study_folder)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        
    SEED = 9855502
    count = 0
    while True:
        study_name = f"HPO_{count}"
        study_path = os.path.join(study_folder, study_name)
        storage_name  = "sqlite:///{}.db".format(study_path)
        try:
            study = optuna.create_study(direction='maximize',study_name=study_path, 
                                        storage=storage_name,load_if_exists=False, 
                                        sampler=optuna.samplers.TPESampler(seed=SEED))
        except optuna.exceptions.DuplicatedStudyError:
            count += 1
        else:
            break;
    return study