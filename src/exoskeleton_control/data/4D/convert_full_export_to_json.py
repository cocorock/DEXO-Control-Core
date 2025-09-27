import scipy.io
import json
import numpy as np
import os
import argparse

def clean_value(value):
    """Recursively cleans MATLAB data types for JSON serialization."""
    # If it's a numpy array
    if isinstance(value, np.ndarray):
        # If it's a structured array (a MATLAB struct)
        if value.dtype.names:
            # If it is 0-dimensional, it's a single struct; convert to dict
            if value.ndim == 0:
                return {name: clean_value(value[name].item()) for name in value.dtype.names}
            # Otherwise, it's an array of structs; convert to list of dicts
            else:
                return [clean_value(v) for v in value]
        # If it's a regular numpy array, convert to a list
        else:
            return value.tolist()
    
    # If it's a scalar struct that has been extracted by .item()
    elif isinstance(value, np.void):
        return {name: clean_value(value[name]) for name in value.dtype.names}

    # Handle scalar types that are wrapped in numpy types
    elif isinstance(value, (np.generic, np.number)):
        return value.item()

    # Return other Python-native types as is
    elif isinstance(value, (int, float, str, list, dict)):
        return value
    
    elif value is None:
        return None

    # Fallback for any other unknown type
    else:
        return str(value)

def main():
    """Main function to load the full .mat file, convert it, and save as .json."""
    parser = argparse.ArgumentParser(description='Convert a subject-specific .mat export file to .json.')
    parser.add_argument('subject_id', type=str, help='The subject ID (e.g., 35) used in the filename.')
    args = parser.parse_args()

    subject_id = args.subject_id
    output_dir = os.path.join('Gait Data', '4D')
    base_filename = f'gait_analysis_export_subject{subject_id}'
    
    mat_file_path = os.path.join(output_dir, f'{base_filename}.mat')
    json_file_path = os.path.join(output_dir, f'{base_filename}.json')

    if not os.path.exists(mat_file_path):
        print(f"Error: Input file not found at '{mat_file_path}'")
        print("Please ensure the subject ID is correct and the MATLAB script has been run.")
        return

    print(f"Loading .mat file from: {mat_file_path}")
    try:
        mat_contents = scipy.io.loadmat(mat_file_path, squeeze_me=True)
    except Exception as e:
        print(f"Error loading .mat file: {e}")
        return

    main_key = [k for k in mat_contents.keys() if not k.startswith('__')][0]
    gait_data_export = mat_contents[main_key]

    print("Converting MATLAB struct to Python dictionary...")
    converted_data = clean_value(gait_data_export)

    print(f"Saving JSON file to: {json_file_path}")
    try:
        with open(json_file_path, 'w') as json_file:
            json.dump(converted_data, json_file, indent=4)
        print("Conversion successful!")
    except Exception as e:
        print(f"Error writing JSON file: {e}")

if __name__ == '__main__':
    main()
