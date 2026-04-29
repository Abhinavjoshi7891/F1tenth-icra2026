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
    result = {}
    if hasattr(msg, 'get_fields_and_field_types'):
        for field in msg.get_fields_and_field_types().keys():
            val = getattr(msg, field)
            full_key = f'{prefix}.{field}' if prefix else field
            result.update(_msg_to_flat_dict(val, full_key))
    elif isinstance(msg, (int, float, bool, str)):
        result[prefix] = msg
    # arrays and compound types with no sub-fields are silently ignored
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

        df = pd.DataFrame(rows)

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
        print(f'  {len(df)} rows from {topic}')

    dfs = list(topic_dfs.values())
    merged = dfs[0].sort_values('timestamp_ns')
    for df in dfs[1:]:
        merged = pd.merge_asof(
            merged,
            df.sort_values('timestamp_ns'),
            on='timestamp_ns',
        )
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
    result = pd.concat(dataframes, ignore_index=True)
    print(f'Total datapoints: {result.shape[0]}')
    return result
