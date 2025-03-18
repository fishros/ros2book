# Hands-On ROS 2 for Robotics Programming

This repository contains the documentation and resources for the book **Hands-On ROS 2 for Robotics Programming**. The book provides a comprehensive guide to developing intelligent robots using ROS 2, covering both theoretical concepts and practical implementations.

## About the Book

Robot development is a complex system engineering task. ROS 2 offers strong support for intelligent robot development, greatly improving development efficiency. This book introduces ROS 2's basic concepts, communication mechanisms, libraries, and tools, helping readers get started with ROS 2 robot development. It then guides readers through practical tasks such as:

- Robot modeling and simulation
- Mapping and navigation
- Custom controller and planner development

In the real-robot section, readers will build a physical ROS 2-based robot, bridging the gap between simulation and reality. The book also delves into advanced ROS 2 usage, laying a solid foundation for further practical robot development. Examples are implemented in both C++ and Python, with additional explanations of C++ features, Git, multithreading, and callback functions.

## Setting Up the Environment

To follow along with the examples in the book, set up your environment as described below.

### Installation

Install the required dependencies:

```bash
pip install Sphinx sphinx-autobuild sphinx_rtd_theme recommonmark sphinx_markdown_tables
```

### Building and Viewing Documentation

1. Run the following command to build and serve the documentation:

    ```bash
    sphinx-autobuild source build/html
    ```

2. Open your browser and navigate to the provided URL to view the documentation.

## Dependencies

- **Sphinx**: Documentation generator.
- **sphinx-autobuild**: Automatically rebuilds and serves the documentation.
- **sphinx_rtd_theme**: Read the Docs theme for Sphinx.
- **recommonmark**: Enables Markdown support in Sphinx.
- **sphinx_markdown_tables**: Adds support for Markdown tables in Sphinx.

## Additional Resources

For more information and updates, refer to the book's official website or repository.
