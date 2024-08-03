# SolvED-ALNS

Welcome to the playground of the champion solution for the MOPTA 2021 competition! 🏆 This epic event, hosted by Leigh University, brought the best and brightest minds together to tackle a global operations research challenge. For its 13th anniversary, MOPTA partnered with AIMMS for a worldwide competition extravaganza. Curious about the details? Check out [MOPTA 2021](https://coral.ise.lehigh.edu/~mopta2021/competition).

The challenge? A little something called the **Home Service Assignment, Routing, and Appointment Scheduling (H-SARA) Problem.** Imagine a world where home services are assigned, routed, and scheduled with the precision of a Swiss watch. Sounds easy, right? 😅

Enter team SolvED from the University of Edinburgh! Armed with brainpower and determination, we—Shunee Johnn, Andrés Miniguano-Trujillo, and myself—conquered the competition. 🥇 Dive into our detailed paper with all the test results [here](https://drops.dagstuhl.de/entities/document/10.4230/OASIcs.ATMOS.2021.4).

## High-level description of the approach

Our master plan for solving the H-SARA problem can be boiled down to two main scenarios:

1. **The Day Before D-Day:** We notify our customers about the magical time window when they can expect our service wizards to appear. 🧙‍♂️ For this scenario, we developed a districting model to divide areas into manageable districts.
2. **Showtime:** On the actual service day, we deploy our team of heroes to a chosen set of customers, plotting out the best route to make sure we stick to those time window promises like glue. Our approach here combines Adaptive Large Neighborhood Search (ALNS) with an exact model to ensure optimal performance.

Furthermore, we created a web application for no-code usage, making it easier than ever to utilize our solution! 🚀

And that's how we became the reigning champs of MOPTA 2021! 🎉

## How to Run the Model

Ready to dive in and see the magic in action? Here's how you can run our model using Docker:

1. **Clone the Repo:**
   ```sh
   git clone https://github.com/your-repo/solved-alns.git
   cd solved-alns
   ```
2. **Build the Docker Image:**
   ```sh
   docker build -t solved-alns .
   ```
3. **Run the Docker Container:**
   ```sh
   docker run -p 5000:5000 solved-alns
   ```
