"""
Utility functions for dictionary-based file I/O operations.

This module provides functions to:
- Load and save YAML configuration files
- Expand file paths
- Perform dictionary-related operations
"""

import os
from typing import Dict, Any, Optional, Tuple

import yaml


def full_path(file_path: str) -> str:
    """
    Expand and resolve the full path to a file.

    Args:
        file_path (str): Relative or user-referenced file path.

    Returns:
        str: Fully resolved absolute file path.

    Examples:
        >>> full_path('~/config.yaml')
        '/home/username/config.yaml'
    """
    return os.path.expanduser(os.path.abspath(file_path))


def load_yaml(config_path: str) -> Dict[str, Any]:
    """
    Load a YAML configuration file.

    Args:
        config_path (str): Path to the YAML configuration file.

    Returns:
        Dict[str, Any]: Parsed configuration dictionary.

    Raises:
        FileNotFoundError: If the configuration file does not exist.
        yaml.YAMLError: If there's an issue parsing the YAML.

    Examples:
        >>> config = load_yaml('scenario.yaml')
        >>> print(config['scenario']['name'])
    """
    try:
        with open(full_path(config_path), "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        raise FileNotFoundError(f"Configuration file not found: {config_path}")


def save_yaml(
    config_dict: Dict[str, Any], config_path: str, sort_keys: bool = True
) -> None:
    """
    Save a dictionary to a YAML file.

    Args:
        config_dict (Dict[str, Any]): Dictionary to save.
        config_path (str): Destination file path.
        sort_keys (bool, optional): Whether to sort dictionary keys. Defaults to True.

    Raises:
        IOError: If there are issues writing to the file.

    Examples:
        >>> save_yaml({'scenario': {'name': 'test'}}, 'output.yaml')
    """
    try:
        with open(full_path(config_path), "w", encoding="utf-8") as file:
            yaml.dump(config_dict, file, sort_keys=sort_keys)
    except IOError as e:
        raise IOError(f"Error saving configuration to {config_path}: {e}")


def get_nested(
    dictionary: Dict[str, Any], *keys: str, default: Optional[Any] = None
) -> Optional[Any]:
    """
    Safely retrieve a nested dictionary value.

    Args:
        dictionary (Dict[str, Any]): Source dictionary.
        *keys: Nested keys to traverse.
        default (Optional[Any], optional): Default value if key not found. Defaults to None.

    Returns:
        Optional[Any]: Retrieved value or default.

    Examples:
        >>> config = {'a': {'b': {'c': 42}}}
        >>> get_nested(config, 'a', 'b', 'c')
        42
        >>> get_nested(config, 'x', 'y', default='Not found')
        'Not found'
    """
    for key in keys:
        if not isinstance(dictionary, dict):
            return default
        dictionary = dictionary.get(key, default)
        if dictionary == default:
            break
    return dictionary


def convert_dict_to_str(dictionary: Dict[str, Any], indent: int = None) -> str:
    """
    Convert a dictionary to a formatted string using yaml.

    Args:
        dictionary (Dict[str, Any]): The dictionary to convert.
        indent (int, optional): Indentation level. Defaults to 2.

    Returns:
        str: The formatted string.
    """
    return yaml.dump(dictionary, indent=indent)


def convert_str_to_dict(
    string: str,
) -> Dict[str, Any]:
    """
    Convert a string to a dictionary using yaml.
    
    Args:
        string (str): The string to convert.\
        
    Returns:
        Dict[str, Any]: The converted dictionary.
    """
    try:
        return yaml.safe_load(string)
    except yaml.YAMLError as e:
        return {"error": str(e)}


def round_dict_values(d: Dict[str, Any], num_decimals: int = 3) -> Dict[str, Any]:
    """
    Round the values of a dictionary to a certain number of decimal places.

    Args:
        d (Dict[str, Any]): The dictionary.
        num_decimals (int): The number of decimal places.

    Returns:
        Dict[str, Any]: The dictionary with rounded values.
    """
    for k, v in d.items():
        if isinstance(v, str):
            d[k] = v
            continue
        if isinstance(v, dict):
            d[k] = round_dict_values(v, num_decimals)
        elif isinstance(v, list):
            d[k] = [round(x, num_decimals) if not isinstance(x, str) else x for x in v]
        else:
            d[k] = round(v, num_decimals)
    return d


def build_tree_from_dict(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Reconstruct the tree structure from the dictionary.
    This assume a dictionary where each key is a node and the value is a
    dictionary with a "parentName" key pointing to the parent node.
    Args:
        data (Dict[str, Any]): The dictionary.

    Returns:
        Dict[str, Any]: All the roots of the tree.
        Dict[str, Any]: The tree structure.
    """
    tree = {}  # Contain each nodes and their children
    roots = []
    for node, info in data.items():
        parent = info.get("parentName", None)
        if parent is None:
            roots.append(node)
        else:
            if parent not in tree:
                tree[parent] = []
            tree[parent].append(node)
    return roots, tree


def convert_tree_into_proper_dict(
    root: str, tree: Dict[str, Any], data: Dict[str, Any], relevant_keys: Tuple[str]
) -> Dict[str, Any]:
    """
    Convert a tree structure into a proper dictionary that
    can easily display the tree.

    Args:
        root (str): The roots of the tree.
        tree (Dict[str, Any]): The tree structure
        data (Dict[str, Any]): The dictionary containing each nodes and their values
        relevant_keys (Tuple[str]): The keys to keep in the final dictionary

    Returns:
        Dict[str, Any]: The tree structure in a readable format.
    """
    root_val = data[root]
    res = {k: root_val[k] for k in relevant_keys if k in root_val}
    if root in tree:
        for child in tree[root]:
            res[child] = convert_tree_into_proper_dict(child, tree, data, relevant_keys)
    return res
