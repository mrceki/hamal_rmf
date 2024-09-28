Build the dockerfile:
# Docker Setup for Altinay RMF

## Building the Docker Image

```bash
docker build -t altinay_rmf_iron .
```

## Using the Bash Script to Manage the Container

### First-Time Setup

Make the script executable:

```bash
chmod +x start-docker-host.sh
```

Run the script to create/start/attach a container:

```bash
./start-docker-host.sh altinay_rmf altinay_rmf_iron:latest false  # Use 'true' if with GPU
```

### Subsequent Runs

Simply run the script with:

```bash
./start-docker-host.sh altinay_rmf
```