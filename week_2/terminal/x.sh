sudo mkdir -p ~/logs
sudo mv /root/*log* ~/logs/
# mkdir = make directory
# -p = ensures that the command doesn’t fail if the directory already exists
# mv = move files or directories from one location to another
# /root/*log* = matches all files in the /root directory that contain "log" in their names
# ~/logs/ = the destination directory in the user's home directory