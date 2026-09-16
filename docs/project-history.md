# Project History

## Acknowledgements

This project exists because of an invitation.

Back in 2020 I started working on a tattooing robot, invited by my robotics
teacher, **Olmer García**. The initiative originally came from **Alis
Paraquiva**, a chemistry teacher, and **Valeria Jorge**, a chemistry student.
They had the first idea and brought it to Olmer, who then brought it to me.

The plan was to build a robot inside a multidisciplinary team, for precision
tattooing — printing conductive synthetic substrates, with possible applications
in medicine.

**Thank you to Olmer García, Alis Paraquiva and Valeria Jorge.** Without that
initiative and that first invitation, this project would probably never have
started.

---

## 2020: the first attempt

I was fairly motivated at the time, so I began developing the robot and its
description in URDF and SDF. One of the early stages is in this video:

<https://www.youtube.com/watch?v=es29rG9TQoY>

Later I also started on some of the robot's control packages, though I never
published videos of that part.

Then COVID-19 arrived and I had to leave the city for financial reasons. I spent
a long time away and disconnected from my projects, and little by little this one
ended up forgotten — one more repository on my GitHub.

## 2026: picking it up again

The project stayed unfinished for a long time. Recently, going through my
GitHub, I came across it again and decided to take it up.

This time I developed it leaning heavily on AI: code generation, nodes, parts of
the architecture, and various implementation tasks. It was genuinely interesting
to return to a project I had left behind in 2020 and see how far I could take it
with today's tools.

I no longer have the physical robot, so for now the project is focused entirely
on simulation.

Although much of the current development was AI-assisted, I am personally proud
and happy to have been able to pick up a project that had been unfinished since
2020 and carry it a little further.

I hope you like it, and that it turns out to be interesting for people working
with ROS, robotics and simulation.

— [MrDavidAlv](https://github.com/MrDavidAlv)

---

## What changed between the two

The 2020 work was the description and the beginnings of a control stack, on
ROS 1. What exists now is a ROS 2 Humble port of that description plus the whole
chain that was missing: artwork to ink mask to toolpath, inverse kinematics,
rigid body dynamics, joint control, and an error budget that ties them together.
The [mathematical model](./mathematical-model/) documents that chain, and
[Migration Notes](../README.md#migration-notes) covers what the port itself
involved.
