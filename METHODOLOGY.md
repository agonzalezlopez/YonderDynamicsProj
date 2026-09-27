# Methodology

## 1. How to run it

A reviewer should be able to follow this from a fresh clone without asking you anything. Test it yourself in a clean checkout before you submit.

-1. Download Python Version 3.14.5 and install dependencies

-2. Create a Folder where the project will reside
    + If you are not signed into Git Bash run these commands and replace the characters in the String with your actual information:
        -git config --global user.name "Your Name"
        -git config --global user.email "you@example.com"
    + Using Git Bash, go into that directory using "cd" and "ls"

-3. Create a repository for that project in Github
    + Copy the Official code on the Github from the green "Code" dropdown as HTTPS
    + Input Command into Git Bash: "git clone <'LINK THAT WAS COPIED'>"

-4. After cloning input the follwing commands, without the "+" and replace the https link with your repository link, by grabbing the HTTPS from YOUR github repo instead of the Official code's Github: 
    + git remote set-url origin https://github.com/<your-username>/<your-repo>.git
    + git push -u origin main

-5. Ensure you are inside the repositories root, using Git Bash make sure you are inside the "autonomous-take-home" folder, by running "pwd". If not use "cd" to go into the directory

-6. Install packages by running this line on Git Bash: pip install -r requirements.txt

-7. To Start the simulation run: py sim/launch.py --scenario open
        + To visualize run: py sim/launch.py --scenario boulders --visualize
## 2. Thought process
Your approach and the reasoning behind it, in bullet points.

* grid_utils.py: 
    - Initial approach was figuring out the math behind the conversion between World_to_cell. Issue was that I was using the wrong rounding function causing the values to be off by 1. Final Approach was switching to floor() making values good. 
    - For inflate(), initial approach was inflating and searching in the grid copy causing other cells to also be part of the inflation. Final Approach was searching through the original grid and inflating the copy grid, using the distance formula to mark cells within the radius as occupied.

* Search.py: 
    - For the Heuristic function, Initial Approach was only using side to side movements before applying diagonal movement. Final approach was to use diagonal movement as much as possible until no longer possible and would finish off using horizontal or vertical movement
    - For A* function, Initial Approach was to figure out general structure before starting, primarily figuring out how to move past cells into previously_seen list as well as the neighbors. Final Approach was to use the Heap data structure for the low cost path, then once we reach the goal, we trace back using the list to reconstruct the path to send as the most low cost path.

* replan.py: 
    - Initial approach was creating a loop on which path took the shortest. Final approach used the distance formula to get the shortest distance and saving those coordinates
    -For path_isValid(), mostly was checking if the current path is valid from current position, only had to check if it was within bounds and that it was not occupied

* planner_node.py: 
    - Intial Approach is using the A* algorithm to detect best path using known values. Final Approach is to constantly replanning every 2 seconds if there is no path, whenever the goal is blocked.
    - For Noise simulation, I used a threshold for the confidence in that cell in order to make my next decision if replan was necessary because that cell was occupied

## 3. Known limitations

What doesn't work, and what you'd do next. Being upfront counts in your favor.

* Currently does not fully handle the noise in the simulation, it still considers some cells to be occupied even though they don't exist. However it is still able to reach goal with fewer phantom replans than previous modifications

* What I would do in the future would be figuring out if I should only consider higher confidence threshold or need other helper functions to reduce the amount of phantom plans there are.