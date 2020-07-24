# Colibrí: GNC 2D models

In this repository you'll be able to find the matlab models and scripts developed by the GNC subsystem (from the [Colibrí Mission](https://www.colibrimission.com/)) for a 2-dimensional analysis of our CubeSat's dynamics, a brief description of the dynamics used can be found in our [theory notebook](Theory.ipynb).

The Simulink models were made with Matlab version 2019b, so you'll need at least that version to open them.

Repository directory

    ~colibri_gnc_2d
    ├───code
    │   ├───orbit_box.slx
    │   ├───orientation_box.slx
    │   │───sat_orbit_orientation.slx
    │   │───statespace_2DOF_translation.m
    │   └───plot_orbit.m
    ├───outputs
    │   └───.gitkeep
    ├───.gitignore
    └───README.md

## Instructions

You should run the **sat_orbit_orientation.slx** Simulink file. We have been using *ode45 (Dormand-Prince)* solver, and have observed good results by setting the max. step size between 0.01 and 0.1 seconds.

After running the Simulink model, a structure called **out** should appear in your workspace. This variable contains all of the Simulation results. Once you have these, you can plot the data and save an animation into an MP4 file inside of the **outputs** folder by calling the **plot_orbit.m** script.

When calling the script you just have to input the **out** structure and the desired filename, such as:

    plot_orbit(out,<FILENAME>);

For example, if you want to save the output video with the name ***"example"*** you'll have to run:

    plot_orbit(out,'example');
