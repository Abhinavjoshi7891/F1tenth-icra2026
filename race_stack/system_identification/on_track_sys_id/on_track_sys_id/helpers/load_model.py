import yaml
import os
from .dotdict import DotDict
from .path_resolver import get_package_path

def get_dict(model_name):
    model, tire = model_name.split("_")
    package_path = get_package_path()
    file_path = os.path.join(package_path, 'models', model, f'{model_name}.txt')
    with open(file_path, 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params
