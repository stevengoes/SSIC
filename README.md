## SSIC - Statistical Shape Image Comparison

Several apps for 3D scanning exists but most of them need extra hardware or calibration tools [(link)](https://www.aniwaa.com/best-3d-scanning-apps-smartphones/). Compared to other photogrammetry methods, Shape-from-Silhouette (SfS) is computationally simple [(paper)](https://link.springer.com/referenceworkentry/10.1007%2F978-0-387-31439-6_211) and the scanning procedure can be performed marker-less [(paper)](https://ieeexplore.ieee.org/document/7322186). However, the digital model from SfS is often an inaccurate representation of the real shape. These SfS models are also known as visual hull (VH). But the availability of large-scale anthropometric surveys using 3D body scanning technologies has enabled researchers to predict 3D body shapes from partial inputs, such as a small set of silhouette pictures [(paper)](https://www.inderscienceonline.com/doi/abs/10.1504/IJDH.2016.084581),[(paper)](https://www.tandfonline.com/doi/full/10.1080/19424280.2015.1038308). A statistical shape model (SSM) is designed with the survey models and can be used to morph and fit within the object outline in the silhouette image. This silhouette-based shape estimation will be called the statistical shape image comparison (SSIC) method. It compares projections of the SSM with silhouette images to validate and morph the SSM into the desired 3D shape. This technique has brought potential benefits to the e-commerce of wearable products but might just as well be used in the medical field for wearable devices such as prostheses.

## Motivation

This research is part of a TU Delft Global Initiative project named ”Access to prosthetics thanks to 3D printing and a smartphone app” [(link)](https://www.tudelft.nl/en/2018/3me/using-a-smartphone-as-a-3d-printer/). The goal of the project is to make prosthetic devices easy accessible by automating the design process. A prosthetic hand model with no need for assembly, a parametric socket design, cheap materials for 3DP and a smartphone as a residual limb measuring tool will enable a fast and low-cost production. The process will be an end-to-end solution where the patient will scan his residual limb as input and
a fitting prosthesis will be delivered as output. In between is a black box, where a fitting hand prosthesis is designed by the computer and automatically 3D printed. The project is meant particularly for Colombia.

## Code Summary

Show what the library does as concisely as possible, developers should be able to figure out **how** your project solves their problem by looking at the code example. Make sure the API you are showing off is obvious, and that your code is short and concise.

![alt text](https://github.com/stevengoes/SSIC/blob/master/codeflow.png)

## Installation

Provide code examples and explanations of how to get the project.

## API Reference

Depending on the size of the project, if it is small and simple enough the reference docs can be added to the README. For medium size to larger projects it is important to at least provide a link to where the API reference docs live.

## Tests

Describe and show how to run the tests with code examples.

## Contributors

Let people know how they can dive into the project, include important links to things like issue trackers, irc, twitter accounts if applicable.

## License

A short snippet describing the license (MIT, Apache, etc.)
