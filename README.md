# deformable_explore
This Repository is for me to work at the robot both at home and at work.
There will be a mix of projects in this repo, but the entire point is to have this repository be a box for all the tools that I made so far about the deformable object mainpulation project. 

The repo will be structured as a big tree, and each branch will be found containing different kinds of tools. The structure that I have in mind now is: camera, robots, ROS, and other tools.

## Introduction to the project, and maybe just some thought from me.
If we think of robots as a form of life that we create, we use human beings (ourselves) as a template. I love to see this work in this way. Like look, we are literally trying to carry things from human a bit and a bit to this metal artifact called robots. The logic is intuitive. We are most intuitively transferring working knowledge into the unknown fields, trying to see the transferable success. 

It is believed that robots need perception to work with this physical world. We as human beings are not able to imaging ways to interact with the world without perceptions. Different senses, or modalities combine different information about the world. With only vision, one cannot so easily learn about the surface texture; without smell, the perception of taste is also impaired. 

As the development of robotics progress, the increasing need for automation brings more and more sensors onto robots. In the below section I am going to talk about different sensors on robots and their jobs. In another section, I will be talking about how those modalities are fused together and shaped to the form of the 'world' in the 'mind' of robots.

### Human perception comparing to robot perception
+ Vision: the ability to see, is a sense that's shared amongst a variety of living things on earth. Vision is the sense for the incoming light. A healthy human is equipped with stereo binocular 'cameras', the two eyes that we have. The binocular design gives us a wider field of view, and the perception of depth in the overlapping area. The perception of depth is achieved by recognizing the distance between one feature point in left and right images: if the feature point is far away, the distance between the eye and the feature point is close; on the other hand, if the distance is close, the distance between is far. Without two eyes fully open, our perception of depth got impaired. However, even with one eye closed, some sense of depth still remains. Actually, we can move around and see the difference of moving distance. What if the scene is stationary and monocular? We still can capture the depth information because how it looks like from our experience of the world, for example, looking at a picture we can tell the mountain is behind the buildings. Although it is harder to get a correct answer when we are eliminating the inputs, it is not impossible to do so. Robots can be equipped with a larger variety of visual inputs, enabling the sensing in larger range comparing to visible light. Lidars are also widely used on robots nowadays, which use light pulses to sense distance. 

+ Audition/Sound

+ Touch/Somatosensation: Human has a very delicate organ, skin, as the sensor of touch. A huge network of nerve endings and touch receptors are distributed around in the skin, that returns all the touch sensation, including temperature, friction, pressure, tickle, pain, vibrations, etc. [[1]](#1) In robots, touch is a rather under-developed area, yet is a rapidly moving research area. There are labs developing capacitive and resistive sensors; Gelsight is developed and commercialized recent years and those optic-based sensors are becoming trendy in labs and hobbists because their high precision and plug-and-play features. There are other kinds of force and haptic sensors, but not exhaustively listed here. 

+ Taste/Gustation
+ Smell/Olfaction
+ Balance/Vestibular Sense: Human sense of balance comes from the vestibular system in our ear. Three semicircular canals enables the sense of turning of head in three different directions; the otolith organs detects the change of linear movement. [[2]](#2) Robots can be equipped with accelerometers and gyroscopes to detect the six movements. 

+ Proprioception: is a sense that provides information of muscle tension, joint location, and the movement of the body parts. For example, according to [[3]](#3), human eyeballs are controlled by six extraocular muscles (EOMs), and proprioception is the sense of those six muscles and to tell in which direction is the eyeball pointing at, which is important for locating objects in surrounding environment. Robots can be equipped with proprioception in different ways, and is yet to be rigorously defined. Robotic proprioception is usually achieved by internal sensors like IMU, which typically includes accelerometer and gyroscope that were discussed in the previous point (Balance/Vestibular Sense). Depending on the acutation method, proprioception can be achieved differently. For example, Zou et. al in [[4]] developed a method to determine strength of grip by sensing the fluid pressure in pneumatic actuators to achieve robust manipulation of delicate deformable objects without no embedded sensors on the end effector. 






# Citation
<a id="1">[1]</a> : Groove, “Sense of Touch, Skin Receptors, Skin Sensations, Somatosensory System,” Home Science Tools Resource Center. Accessed: Mar. 10, 2025. [Online]. Available: https://learning-center.homesciencetools.com/article/skin-touch/

<a id="2">[2]</a> : InformedHealth.org [Internet]. Cologne, Germany: Institute for Quality and Efficiency in Health Care (IQWiG); 2006-. In brief: How does our sense of balance work? [Updated 2023 Sep 25]. Available from: https://www.ncbi.nlm.nih.gov/books/NBK279394/

<a id="3">[3]</a> :  R. Blumer, G. Carrero-Rojas, R. R. De La Cruz, J. Streicher, and A. M. Pastor, “Extraocular Muscles: Proprioception and Proprioceptors,” in Reference Module in Neuroscience and Biobehavioral Psychology, Elsevier, 2024, p. B9780443138201000281. doi: 10.1016/B978-0-443-13820-1.00028-1.

<a id="4">[4]</a> :  S. Zou, S. Picella, J. de Vries, V. G. Kortman, A. Sakes, and J. T. B. Overvelde, “A retrofit sensing strategy for soft fluidic robots,” Nat Commun, vol. 15, no. 1, p. 539, Jan. 2024, doi: 10.1038/s41467-023-44517-z.
