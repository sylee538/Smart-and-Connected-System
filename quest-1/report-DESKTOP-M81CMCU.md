# Quest Name
Authors: Rene Colato, Edward Hong, Seung Yeun Lee

2019-09-20

## Summary
This project uses a combination of skills exercises to build a retro clock with an alarm,
pause, and time set features. Our micro is more than capable of realizing an electronic
watch or alarm clock function. It’s probably more powerful than most wrist-worn watches.
In this quest you will wrangle the features of the ESP32 to build a simple alarm clock
with digital output on the alphanumeric display. But we’ll make it even more interesting
by adding mechanical clock hands to indicate passing time.


## Investigative Question
Investigative question: How can you synchronize multiple ESP clocks with each other? Elaborate.

A method where we could synchronize multiple ESP clocks with each other is using a master-slave relationship between ESP chips where the console provides the data to the master and the master passes the information to the other chips. There may be a timing delay that would have to Implemented to get them synched properly to each other. But if that were to fail the other idea would be to have each ESP communicating to the console simultaneously such that all information reaches each time at the same time, but the possible problems we would run into is the method to have each chip speaking to the console reliably.

## Evaluation Criteria



## Solution Design



## Sketches and Photos
<center><img src="./images/example.png" width="70%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo](https://www.youtube.com/watch?v=nJY2S9Rs4o8)



## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
