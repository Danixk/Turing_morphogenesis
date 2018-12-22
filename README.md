# Morphogenesis in robot swarms

Authors: Ivica Slavkov, Daniel Carrillo-Zapata, Noemí Carranza, Xaver Diego, Fredrik Jansson, Jaap A. Kaandorp, Sabine Hauert, James Sharpe.

Contact: James Sharpe: james.sharpe@embl-barcelona.es 

Science Robotics  19 Dec 2018:

Vol. 3, Issue 25, eaau9178

DOI: 10.1126/scirobotics.aau9178 

[Read article](http://robotics.sciencemag.org/content/3/25/eaau9178)

[Blog post](https://robohub.org/growing-bio-inspired-shapes-with-a-300-robot-swarm/)


## Description

This repository contains the code for the algorithm developed as part of the [Swarm Organ Project](http://www.swarm-organ.eu/) and additionally supported by the Spanish Ministry of Economy and Competitiveness and the [EPSRC Centre for Doctoral Training in Future Autonomous and Robotic Systems (FARSCOPE)](http://farscope.bris.ac.uk/). The code is distributed under the MIT license (see the file *LICENSE* for details).

The following code has been developed specifically for the kilobots [1]. The file *morphogenesis.hex* contains the compiled code of our proposed morphogenesis algorithm, and it is directly available to be uploaded to kilobots. The code was compiled using the [kilolib library](https://github.com/acornejo/kilolib) in its state in June, 2018. For local compilation, please refer to https://www.kilobotics.com/ .

The code has been adapted to the simulator Kilombo [2]. Please refer to https://github.com/JIC-CSB/kilombo to install the simulator and/or understand the syntax.

The algorithm assumes the kilobots have been calibrated for left and right motion. Calibration for forward motion is not required.


## References
1. M. Rubenstein, C. Ahler, R. Nagpal, Kilobot: A Low Cost Scalable Robot System for Collective Behaviors, Proceedings of 2012 IEEE International Conference on Robotics and Automation, 3293-3298 (2012).
1. F. Jansson, M. Hartley, M. Hinsch, I. Slavkov, N. Carranza, T. S. G. Olsson, R. M. Dries, J. H. Grönqvist, A. F. Marée, J. Sharpe, J. A. Kaandorp, V. A. Grieneisen, Kilombo: a Kilobot simulator to enable effective research in swarm robotics, arXiv (2015).


## Credits

This application uses Open Source components. You can find the source code of their open source projects along with license information below. We acknowledge and are grateful to these developers for their contributions to open source.

Project: Kilombo [https://github.com/JIC-CSB/kilombo](https://github.com/JIC-CSB/kilombo)

Copyright (c) 2015, University of Amsterdam (Amsterdam, The Netherlands), John Innes Centre (Norwich, United Kingdom), Centre for Genomic Regulation (Barcelona, Spain)

License (MIT) [https://github.com/JIC-CSB/kilombo/blob/master/LICENSE](https://github.com/JIC-CSB/kilombo/blob/master/LICENSE)
