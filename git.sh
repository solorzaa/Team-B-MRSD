#!/bin/bash

git add . 
git status
read -p "Write your commit message: " mesg
git commit -m "$mesg" 
git pull origin master

git push origin master
#firefox https://github.com/mehtajaghvi
exit


