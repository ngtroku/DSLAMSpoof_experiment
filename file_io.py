import pandas as pd

def load_reference(csv_file_name):
    df = pd.read_csv(csv_file_name)
    x, y, z = df['x'].to_numpy(), df['y'].to_numpy(), df['z'].to_numpy()
    return x, y, z

def load_reference_df(csv_file_name):
    df = pd.read_csv(csv_file_name)
    return df