Intelligent_Wheelchair-
=======================
This file explains what you have to do to set up a local repo for the Intelligent_Wheelchair remote repo.

- mkdir [localRepoName]
- cd [localRepoName]
- git init 
# set up the necessary files for git

- git remote add origin [ssh key]  for me it is "git@github.com:Samantoutita/Intelligent_Wheelchair-.git"

#To create a remote named origin pointing at out intelligent_wheelchair repo
First pull changes from master 

- git pull origin master
says fetch from master and merge with origin 
We don't want to work on the master branch so we will always create branches and merge later

#Create a new branch
-git branch [newBranchName]

#Switch to the new branch
-git checkout [newBranchName]

#Work on the project and modify the files
- git add [fileNameModified]

- git commit -m "Insert your message here"

# and then push the branch to github
- git push origin [newBranchName]


