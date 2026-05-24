import yaml
import os
from .path_resolver import get_package_path

def save(model, overwrite_existing=True, verbose=False):
  package_path = get_package_path()
  
  # Ensure the directory exists
  model_dir = os.path.join(package_path, "models", model['model_name'])
  if not os.path.exists(model_dir):
      os.makedirs(model_dir)
      
  file_path = os.path.join(model_dir, model['model_name'] + "_" + model['tire_model'] + ".txt")
  
  if os.path.isfile(file_path):
    if (verbose): print("Model already exists")
    if overwrite_existing:
      if (verbose): print("Overwriting...")
    else:
      if (verbose): print("Not overwriting.")
      return 0

  try:
    model = model.to_dict()
  except:
    model = model

  # Write data to the file
  with open(file_path, "w") as f:
      print(f"\033[32m[INFO] MODEL IS SAVED TO: {file_path}\033[0m")
      yaml.dump(model, f, default_flow_style=False)
