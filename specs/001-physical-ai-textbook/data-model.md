# Data Model: Physical AI & Humanoid Robotics Textbook

## Content Entities

### Course Module
- **Description**: Major section of the textbook covering specific robotics concepts
- **Fields**:
  - id: string (unique identifier, e.g., "module-1")
  - title: string (e.g., "The Robotic Nervous System (ROS 2)")
  - description: string (overview of module content)
  - weeks: array of Week objects (ordered list of weeks in this module)
  - learningObjectives: array of strings (specific skills/concepts to master)
  - prerequisites: array of strings (required knowledge before starting)
  - duration: number (estimated weeks to complete)

### Week
- **Description**: Subsection of a module that covers topics for a specific week
- **Fields**:
  - id: string (unique identifier, e.g., "module-1-week-3")
  - title: string (e.g., "ROS 2 Architecture")
  - description: string (overview of week content)
  - topics: array of strings (specific topics covered)
  - learningObjectives: array of strings (specific objectives for the week)
  - contentPath: string (path to the MDX file)
  - exercises: array of Exercise objects
  - duration: number (estimated hours to complete)

### Learning Objective
- **Description**: Specific skill or concept that students should master
- **Fields**:
  - id: string (unique identifier)
  - description: string (what the student should be able to do)
  - moduleRef: string (reference to parent module)
  - weekRef: string (reference to parent week, if applicable)
  - measurable: boolean (can be assessed)

### Code Example
- **Description**: Practical implementation examples that demonstrate concepts
- **Fields**:
  - id: string (unique identifier)
  - title: string (brief description of the example)
  - description: string (explanation of what the code does)
  - code: string (the actual code content)
  - language: string (programming language)
  - moduleRef: string (reference to parent module)
  - weekRef: string (reference to parent week)
  - complexity: enum (beginner, intermediate, advanced)
  - runnable: boolean (can be executed in target environment)

### Exercise
- **Description**: Hands-on activities that allow students to apply concepts
- **Fields**:
  - id: string (unique identifier)
  - title: string (brief description of the exercise)
  - description: string (detailed instructions)
  - type: enum (practical, theoretical, coding, analysis)
  - difficulty: enum (beginner, intermediate, advanced)
  - estimatedTime: number (minutes to complete)
  - moduleRef: string (reference to parent module)
  - weekRef: string (reference to parent week)
  - requiredResources: array of strings (tools, hardware, etc. needed)
  - solution: string (optional solution or guidance)

### Hardware Requirement
- **Description**: Specific hardware specifications and requirements needed to complete the course content
- **Fields**:
  - id: string (unique identifier)
  - name: string (hardware component name)
  - description: string (what it's used for)
  - specifications: object (detailed specs like CPU, RAM, etc.)
  - optional: boolean (whether this is required or optional)
  - moduleRef: string (reference to relevant module)
  - compatibility: array of strings (supported platforms/OS)

### Assessment Guideline
- **Description**: Criteria and methods for evaluating student understanding and progress
- **Fields**:
  - id: string (unique identifier)
  - title: string (e.g., "Module 1 Quiz", "Practical Assignment")
  - description: string (what is being assessed)
  - type: enum (quiz, practical, project, peer-review)
  - criteria: array of strings (specific evaluation criteria)
  - moduleRef: string (reference to relevant module)
  - weight: number (percentage of overall grade, if applicable)

## Content Relationships

### Module Contains Weeks
- One-to-Many relationship between Course Module and Week
- Each module contains 1-4 weeks of content

### Week Contains Learning Objectives
- One-to-Many relationship between Week and Learning Objective
- Each week has 3-8 learning objectives

### Week Contains Code Examples
- One-to-Many relationship between Week and Code Example
- Each week has 2-10 code examples

### Week Contains Exercises
- One-to-Many relationship between Week and Exercise
- Each week has 1-5 exercises

### Module Contains Hardware Requirements
- One-to-Many relationship between Module and Hardware Requirement
- Each module may have 0-10 hardware requirements

### Module Contains Assessment Guidelines
- One-to-Many relationship between Module and Assessment Guideline
- Each module has 1-3 assessment guidelines

## Validation Rules

### Content Validation
- Each module must have a title and description
- Each week must belong to exactly one module
- Each learning objective must be specific and measurable
- Each code example must have a valid language designation
- Each exercise must have an estimated completion time
- Hardware requirements must specify whether they are required or optional

### Structural Validation
- Total course duration must be approximately 13 weeks
- Module 1 (ROS 2) must come before Module 2 (Gazebo/Unity)
- Prerequisites must be satisfied before accessing advanced content
- All content paths must resolve to valid MDX files
- Navigation hierarchy must follow module → week → topic structure

### Quality Validation
- All code examples must be tested and verified
- All exercises must have clear instructions
- Learning objectives must align with module/week content
- Hardware requirements must be current and accurate
- Assessment guidelines must be fair and achievable