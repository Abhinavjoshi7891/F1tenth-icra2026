import yaml
import os


def save(model, overwrite_existing=True, verbose=False):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    folder_path = os.path.join(script_dir, '..', 'models', model['model_name'])
    os.makedirs(folder_path, exist_ok=True)
    file_path = os.path.join(folder_path, model['model_name'] + '_' + model['tire_model'] + '.txt')
    if os.path.isfile(file_path):
        if verbose:
            print('Model already exists')
        if not overwrite_existing:
            if verbose:
                print('Not overwriting.')
            return 0

    try:
        model = model.to_dict()
    except AttributeError:
        pass

    with open(file_path, 'w') as f:
        yaml.dump(model, f, default_flow_style=False)

    print(f'Saved model to: {file_path}')
