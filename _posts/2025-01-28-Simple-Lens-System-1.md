---
title: Simple Lens System 1
date: 2025-01-28
categories:
  - simple lens system
tags:
  - optical_engineering
  - lens
header:
  teaser: /assets/images/simple_lens_system_1/diagram.png
---
## Project Brief
The aim of this project is to create a simple lens system (for now just one lens piece) in OSLO EDU and optimize its properties to focus light onto an image plane. Below are the starting parameters of the lens system.

| Surface | radius of curvature (mm) | thickness (mm) | focal length (mm) | glass type |
| ------- | ------------------------ | -------------- | ----------------- | ---------- |
| object  | infinity                 | infinity       | —                 | —          |
| lens    | 50                       | 5              | 50                | BK7        |
| image   | infinity                 | —              | —                 | —          |

The aperture diameter at each surface should also be 10 mm.

I will keep the thickness and focal lengths fixed, only varying the curvature of the lens surface to focus light onto the image plane.


## Lens System Setup
Setting up the lens system in OSLO EDU requires creating surfaces for each surface the light rays interact with. By default, OBJ (the object), ARF (the aperture radius field), and IMS (the imaging surface) will be present. The ARF is the surface light from the object is incident on, so this should be set for the first surface of the lens. The back surface of the lens becomes surface 2 and the image plane becomes surface 3.
![](/assets/images/simple_lens_system_1/diagram.png)
For simplicity, I choose the catalogue lens MG01LQP001 and adjusted the values to those in the brief. I then changed the glass material of surface 1 to N-BK7 found in the Schott catalogue. I then select _Solved (S)_ for each surface to solve for the aperture radius as the beam passes through each surface. 
![](/assets/images/simple_lens_system_1/20250128142754.png)

There are several ways to evaluate the performance of the lens system. The spot diagram shows the spot of light on the image plane.
![](/assets/images/simple_lens_system_1/20250128142929.png)
The large spot radius compared to the diffraction limit shows this spot is aberration limited and not diffraction limited. That means we can optimize the system's properties to reduce the spot size. This is expected since the focal length of the system is 96.75 mm, much further back than the image plan at 50 mm.

The aberration analysis plots show significant spherical aberration (values are close to 50 mm). There's also some chromatic shift which apparently is expected for a single BK7 lens.
![](/assets/images/simple_lens_system_1/20250128143336.png)
This aberration can be better seen in the wavefront analysis, showing significant wavefront magnitude on the outer edge.
![](/assets/images/simple_lens_system_1/20250128143423.png)


## Lens System Optimization 1
To use OSLO EDU's built-in optimization tools, we need to select an error function (aka a merit function). For a single  element system, I think we can use the Singlet error function. It only has two parameters:
- py is the ray height above the axis
- SA3 is the 3rd term of the primary spherical aberration
![](/assets/images/simple_lens_system_1/20250128144025.png)

Next we need to define our variables. Select the _Variables_ button on the _Surface Data_ editor. The values shown are the curvature which is inversely proportional the the radius of curvature:
$$ k = \frac{1}{R} $$
I set _k_ between 0 and 10 so _R_ was limited to > 0.1 mm. Then click the tick mark to save. Now a V appears next to the curvature of surface 2 (the front surface). 

Then select _Iterate_ under the _Optimize_ menu and press enter. The result is a reduced curvature to 28 mm.
![](/assets/images/simple_lens_system_1/20250128145036.png)
The spot diagram shows much better performance, though the system is still aberration limited since the spot radius is about 50 times larger than the diffraction limit.
![](/assets/images/simple_lens_system_1/20250128145122.png)
The aberration analysis plots also show improvement. While the lines in the spherical aberration plot are more curved, the horizontal axis scale is reduced by 25 times .
![](/assets/images/simple_lens_system_1/Pasted image 20250128145247.png)
The wavefront analysis also show a more balanced wavefront when comparing the colour axis scale to before.
![](/assets/images/simple_lens_system_1/20250128145415.png)

## Next Steps
In the next blog post, I'll aim to improve the optimization by using different algorithms and weights. Then later, I will attempt to add a second lens to improve performance.