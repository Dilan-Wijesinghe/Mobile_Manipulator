I. Explanation

IMPORTANT NOTE:

If you would like to run the code, you MUST change directory into the appropriate titled directory.
For example, if you want to run the code for 'best', cd into 'best' and run python3 best.py.

The generic way of running the code is:
cd into '<dir_name>'
run python3 '<dir_name>.py'
Plug results into Coppelia Sim Scene 6

The following package includes three directories, 'best', 'overshoot', and 'newtask'. 
They reside in a directory called 'results' and include the following:
- Two CSV files
    - The first is titled <dir_name>.csv. This is the csv used to run the code in Coppelia
    - The second is titled <dir_name>_err.csv. This is the X error recorded during execution
- Two PNG files
    - The first is an image of the Error Data, including the X,Y,Z,Roll,Pitch and Yaw errors
    - The second is an image of the Twist error
- One Log file under <dir_name>.log
- One mp4 file, showing off a the moving robot in Coppelia sim with the accompanying csv 

II. Results
The task is described as the following:
- The robot moves from its inital position to the position of the block
- The end effector dips down and grabs the block
- The end effector holds the block and the chassis begins to move towards the final location
- The robot gets to the final location and drops off the block at its final position

The results of the code are as follows. 

For best, the robot does the task with minimal error. There is no overshooting here. These 
results were achieved by finetuning the Kp and Ki values. The robot is also offset by (-1,1).

For overshoot, the robot does the task with some error. It overshoots a bit earlier on and then
manages to bring it back on track. These results were achieved by finetuning the Kp and Ki values
so that Kp was descreased and that Ki was increased. The robot is also offset by (-1,1)

For newtask, the robot does the task with some error that eventually peters off. 
These results were achieved by finetuning the Kp and Ki values. The robot is also offset by (-1,1).
The block's inital and final position are tweaked here in newtask.
Inital: (x,y) = 1, 0.5
Goal: (x,y) = 2, -2

III. Discussion

There are a few surprising results in the errors graphs. The Twist error graphs look fairly normal, 
with the exception of the newtask. I believe this error is produced by the new position of the 
block, as well as error not being tuned as much for this task. 
The error graphs display quite a lot of error for but the order of magnitude is always small, 
and the robots always grab the block, so the results are a bit confusing. 