## End to end:
The combination of deep neural network models and reinforcement learning algorithms can make it possible to learn policies for robotic behaviors that directly read in raw sensory inputs, such as camera images, effectively subsuming both estimation and control into one model. However, real-world applications of reinforcement learning 
must specify the goal of the task by means of a manually programmed reward function

```bash
https://arxiv.org/abs/1904.07854
```
This paper introduce a method removing the need for manual engineering of reward specifications
a  approach for learning skills without manually engineered rewards

---


## Wodel model
Physical Object Understanding with a Physically Controllable World Model

Building computational models that exhibit similar capabilities—learning about objects and their physics through vision and interaction 
Think of physical prompting as:


---

## Physical prompting:
You do not tell the robot with words. You show the robot what to do once.

```bash
(1) You show: pick cube
(2) You show: move cube
(3) You show: put cube in bowl
(4) Robot watches the example
(5) Robot tries the same task
```
### Physical prompting versus training:

Training:
```bash
Training > Show hundreds of examples > Change model weights
```

Physical prompting:
```bash
Physical prompting > Show hundreds of examples >  Show one short example > No weight change > Robot acts immediately

```

