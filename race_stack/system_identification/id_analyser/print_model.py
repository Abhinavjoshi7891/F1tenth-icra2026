#!/usr/bin/env python3
"""
Print all parameters for a vehicle model. Run:
    python3 print_model.py
or:
    python3 print_model.py --model_name MYCAR_pacejka
"""
import fire
from helpers.load_model import get_dotdict

model_name_ = 'MYCAR_pacejka'


def main(model_name: str = model_name_):
    print(f'Loading model {model_name}...')
    model = get_dotdict(model_name)
    for name, value in model.items():
        print(f'  {name}: {value}')


if __name__ == '__main__':
    fire.Fire(main)
