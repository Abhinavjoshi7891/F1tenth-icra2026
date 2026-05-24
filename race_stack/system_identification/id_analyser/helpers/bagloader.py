import os
import sys
import yaml
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _detect_storage_id(bag_path):
    meta_path = os.path.join(bag_path, 'metadata.yaml')
    if os.path.exists(meta_path):
        with open(meta_path, 'r') as f:
            meta = yaml.safe_load(f)
        return meta['rosbag2_bagfile_information']['storage_identifier']
    for fname in os.listdir(bag_path):
        if fname.endswith('.mcap'):
            return 'mcap'
    return 'sqlite3'


def _msg_to_flat_dict(msg, prefix=''):
    prefix = str(prefix)  # kill numpy.str_ propagation through f-string formatting
    result = {}
    if hasattr(msg, 'get_fields_and_field_types'):
        for field in msg.get_fields_and_field_types().keys():
            field = str(field)  # force Python str — rclpy may return numpy.str_
            val = getattr(msg, field)
            full_key = f'{prefix}.{field}' if prefix else field
            result.update(_msg_to_flat_dict(val, full_key))
    elif isinstance(msg, (int, float, bool, str)):
        result[prefix] = msg
    elif hasattr(msg, 'item') and np.ndim(msg) == 0:
        # numpy scalar (np.float64, np.int32, etc.) — convert to Python native
        result[prefix] = msg.item()
    # arrays, sequences, and other compound types are silently ignored
    return result


def get_bag_df(bag_path, field_dict):
    print(f'Loading bag {bag_path}...')
    storage_id = _detect_storage_id(bag_path)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    topic_rows = {topic: [] for topic in field_dict}

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name not in field_dict:
            continue
        msg_type_str = topic_types.get(topic_name)
        if msg_type_str is None:
            continue
        msg_class = get_message(msg_type_str)
        msg = deserialize_message(data, msg_class)
        flat = _msg_to_flat_dict(msg)
        flat['timestamp_ns'] = timestamp_ns
        topic_rows[topic_name].append(flat)

    topic_dfs = {}
    for topic, fields in field_dict.items():
        rows = topic_rows.get(topic, [])
        if not rows:
            print(f'ERROR: no messages for topic {topic}')
            sys.exit(1)

        all_keys = list(dict.fromkeys(k for row in rows for k in row.keys()))
        col_data = {}
        for k in all_keys:
            col_vals = []
            for r in rows:
                v = r.get(k)
                if hasattr(v, 'item') and callable(v.item):
                    try:
                        v = v.item()
                    except Exception:
                        pass
                col_vals.append(v)
            col_data[str(k)] = col_vals

        print(f"  [DEBUG] Attempting pd.DataFrame construction for '{topic}' ({len(rows)} rows, {len(all_keys)} cols)...")

        try:
            df = pd.DataFrame(col_data)
            print(f"  [DEBUG] SUCCESS: pd.DataFrame created for '{topic}'.")
        except Exception as e:
            print(f"\n[CRASH LOG] DataFrame construction failed for topic: {topic}")
            print(f"[CRASH LOG] Error: {e}")
            print("[CRASH LOG] --- DETAILED COLUMN TYPE INSPECTION ---")
            for k, v_list in col_data.items():
                types = set(type(x).__name__ for x in v_list)
                print(f"  -> Column '{k}' contains types: {types}")
                for idx, val in enumerate(v_list):
                    if isinstance(val, np.ndarray):
                        print(f"      !!! FOUND np.ndarray in '{k}' at row {idx} !!! Shape: {val.shape}, Dtype: {val.dtype}")
                        break
                    elif isinstance(val, list):
                        print(f"      !!! FOUND list in '{k}' at row {idx} !!!")
                        break
                    elif type(val).__module__ == 'numpy':
                        print(f"      !!! FOUND rogue numpy type {type(val)} in '{k}' at row {idx} !!!")
                        break
            sys.exit(1)

        # rename Float64 'data' → 'topic.data' to match analysis script convention
        if 'data' in df.columns:
            df.rename(columns={'data': topic + '.data'}, inplace=True)
            keep_fields = [f if f != 'data' else topic + '.data' for f in fields]
        else:
            keep_fields = fields

        keep_cols = ['timestamp_ns'] + keep_fields
        df = df[[c for c in keep_cols if c in df.columns]]
        df.sort_values('timestamp_ns', inplace=True)
        df.reset_index(drop=True, inplace=True)
        topic_dfs[topic] = df
        print(f'  {len(df)} rows from {topic} finalized.')

    dfs = list(topic_dfs.values())
    merged = dfs[0].sort_values('timestamp_ns')
    for df in dfs[1:]:
        try:
            merged = pd.merge_asof(
                merged,
                df.sort_values('timestamp_ns'),
                on='timestamp_ns',
            )
        except Exception as e:
            print(f"\n[CRASH LOG] pd.merge_asof failed!")
            print(f"[CRASH LOG] Error: {e}")
            sys.exit(1)

    merged.dropna(inplace=True)
    merged.reset_index(drop=True, inplace=True)
    return merged


def load_bags(directory, field_dict):
    if not os.path.exists(directory):
        raise ValueError(f'Directory {directory} does not exist')

    bag_dirs = []
    # a ROS2 bag is a directory with metadata.yaml
    if os.path.exists(os.path.join(directory, 'metadata.yaml')):
        bag_dirs = [directory]
    else:
        for entry in os.listdir(directory):
            entry_path = os.path.join(directory, entry)
            if os.path.isdir(entry_path) and os.path.exists(
                os.path.join(entry_path, 'metadata.yaml')
            ):
                bag_dirs.append(entry_path)

    if not bag_dirs:
        raise ValueError(f'No ROS2 bags found in {directory}')

    dataframes = [get_bag_df(p, field_dict) for p in bag_dirs]

    print("  [DEBUG] Attempting pd.concat on all bags...")
    try:
        result = pd.concat(dataframes, ignore_index=True)
        print("  [DEBUG] SUCCESS: pd.concat completed.")
    except Exception as e:
        print(f"\n[CRASH LOG] pd.concat failed!")
        print(f"[CRASH LOG] Error: {e}")
        sys.exit(1)

    print(f'Total datapoints: {result.shape[0]}')
    return result
