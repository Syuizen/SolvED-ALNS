# Use an official miniconda3 image as a parent image
FROM continuumio/miniconda3

# Set the working directory in the container
WORKDIR /app

# Copy the environment.yml file into the container at /app
COPY environment.yml /app/environment.yml

# Create the conda environment
RUN conda env create -f environment.yml

# Make RUN commands use the new environment
SHELL ["conda", "run", "-n", "mopta", "/bin/bash", "-c"]

# Copy the current directory contents into the container at /app
COPY . /app

# Ensure the environment is activated:
RUN echo "conda activate mopta" >> ~/.bashrc
ENV PATH /opt/conda/envs/mopta/bin:$PATH

# Set environment variables for Flask
ENV FLASK_APP=app.py
ENV FLASK_RUN_HOST=0.0.0.0
ENV FLASK_RUN_PORT=5000

# Expose the port that the app runs on
EXPOSE 5000

# Run flask run when the container launches
CMD ["flask", "run"]
