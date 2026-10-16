# Security

The replay does not execute shell commands or load untrusted model output as code. ROS2 deployments should run under a least-privilege user, validate action targets against a map boundary, and keep detector/model artifacts separate from task instructions. Please report vulnerabilities privately to the repository owner before opening a public issue.
