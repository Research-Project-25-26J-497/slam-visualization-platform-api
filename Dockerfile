FROM python:3.12-slim

# Set environment variables for non-buffered output (essential for Docker logs)
ENV PYTHONUNBUFFERED 1
# Set the working directory inside the container
WORKDIR /code

# Copy requirements file and install dependencies
# We install dependencies here to leverage Docker's build cache
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy all local project files into the container
COPY . .