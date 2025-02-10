import os

# Specify the folder and file name
folder_path = "/home/ubuntu/myfolder"
file_name = "myfile.txt"

# Ensure the folder exists
os.makedirs(folder_path, exist_ok=True)

# Create an empty file
file_path = os.path.join(folder_path, file_name)
open(file_path, 'w').close()

print(f"Created empty file: {file_path}")
