# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `1-robotics-textbook` | **Date**: 2025-12-05 | **Plan**: [specs/1-robotics-textbook/plan.md](specs/1-robotics-textbook/plan.md)

## Overview

This document defines the key entities and their relationships for the Physical AI & Humanoid Robotics textbook. This data model primarily represents the structural components of the textbook content rather than a runtime application database.

## Entities

### 1. Module

Represents a single, self-contained unit of learning within the textbook.

-   **Attributes**:
    -   `id`: Unique identifier (e.g., "module-01")
    -   `title`: Title of the module
    -   `learning_outcomes`: List of expected learning achievements
    -   `theory_content`: Markdown content for theoretical explanations
    -   `code_examples`: List of associated Code Examples
    -   `diagrams`: List of associated Diagrams
    -   `simulation_exercises`: List of associated Simulation Exercises
    -   `citations`: List of associated Citations
    -   `glossary_terms`: List of associated Glossary Terms

-   **Relationships**:
    -   `has_many` Code Examples
    -   `has_many` Diagrams
    -   `has_many` Simulation Exercises
    -   `has_many` Citations
    -   `has_many` Glossary Terms

### 2. Code Example

Represents a reproducible source code snippet demonstrating a robotics concept.

-   **Attributes**:
    -   `id`: Unique identifier
    -   `filename`: Name of the code file (e.g., `simple_publisher.py`)
    -   `language`: Programming language (e.g., Python, C++)
    -   `content`: Raw code content
    -   `description`: Explanation of the code's purpose and functionality
    -   `dependencies`: List of external libraries or ROS 2 packages required
    -   `testable`: Boolean, indicates if the code is designed to be executable/tested

-   **Relationships**:
    -   `belongs_to` Module

### 3. Diagram

Represents a visual illustration within the textbook.

-   **Attributes**:
    -   `id`: Unique identifier
    -   `filename`: Path to the image file (e.g., `static/images/robot_architecture.svg`)
    -   `type`: File format (SVG, PNG)
    -   `alt_text`: Alternative text for accessibility
    -   `caption`: Description of the diagram
    -   `source`: Indication of how it was generated (e.g., "AI-generated", "Excalidraw")

-   **Relationships**:
    -   `belongs_to` Module

### 4. Simulation Exercise

Represents a hands-on activity using a simulation environment.

-   **Attributes**:
    -   `id`: Unique identifier
    -   `title`: Title of the exercise
    -   `description`: Instructions and goals for the exercise
    -   `environment`: Required simulation platform (e.g., Gazebo, Isaac Sim)
    -   `assets`: List of required simulation assets (e.g., world files, robot models)
    -   `code_examples`: List of associated Code Examples (e.g., control scripts)
    -   `expected_outcome`: Description of the successful result

-   **Relationships**:
    -   `belongs_to` Module
    -   `has_many` Code Examples

### 5. Citation

Represents a reference to a peer-reviewed or authoritative source.

-   **Attributes**:
    -   `id`: Unique identifier (e.g., APA formatted string or internal ID)
    -   `full_reference`: Full APA-formatted citation string
    -   `inline_text`: Text for inline citation (e.g., (Author, Year))
    -   `url`: Optional URL to the source
    -   `type`: Type of source (e.g., journal article, book, website)

-   **Relationships**:
    -   `belongs_to` Module

### 6. Glossary Term

Represents a definition for a specific robotics or AI concept.

-   **Attributes**:
    -   `term`: The word or phrase being defined
    -   `definition`: The clear, concise explanation of the term
    -   `module_references`: List of modules where the term is primarily used or defined

-   **Relationships**:
    -   `belongs_to` Module (implicitly, through usage)
