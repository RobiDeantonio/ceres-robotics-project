# ceres-robotics-project
Agricultural robotics research project


git init

git remote add origin git@github.com:RobiDeantonio/ceres-robotics-project.git
git remote add origin https://github.com/RobiDeantonio/ceres-robotics-project.git

git branch #Apply double TAB to show actual branches
git fetch --all
git branch -r | grep -v '\->' | while read remote; do git branch --track "${remote#origin/}" "$remote"; done
git pull --all

git checkout main