# Data Model: 4-vla

**Feature**: Module 4 - Vision-Language-Action (VLA)
**Date**: 2025-12-21

## Key Entities

### Voice Command
- **Command Text**: string (required) - The recognized text from speech input
- **Intent Classification**: enum (navigation, manipulation, query, etc.) - The categorized intent of the command
- **Entities Extracted**: array of objects - Named entities identified in the command (objects, locations, etc.)
  - entity_type: string (e.g., "object", "location", "person")
  - entity_value: string - The specific value of the entity
  - confidence: float - Confidence score of the entity recognition
- **Timestamp**: datetime - When the command was received
- **Source Context**: object - Additional context from audio input (volume, clarity, etc.)

### Cognitive Plan
- **Plan ID**: string (required) - Unique identifier for the cognitive plan
- **Input Command**: string - The original voice command that generated this plan
- **Action Sequence**: array of objects - Ordered sequence of robot actions to execute
  - action_type: enum (navigation, manipulation, perception, communication)
  - parameters: object - Specific parameters for the action
  - priority: int - Priority level for the action
  - estimated_time: float - Estimated time to complete the action
- **Context Understanding**: object - Environmental and situational context
  - detected_objects: array of objects - Objects identified in the environment
  - robot_state: object - Current state of the robot
  - goal_state: object - Desired end state
- **Safety Validation**: boolean - Whether the plan passed safety checks

### VLA Pipeline Configuration
- **Configuration Name**: string (required) - Unique identifier for the pipeline configuration
- **Speech Recognition Settings**: object - Configuration for speech processing
  - model_type: string - Type of speech recognition model
  - language: string - Language to recognize
  - noise_threshold: float - Threshold for noise filtering
- **LLM Settings**: object - Configuration for language model integration
  - provider: string - LLM provider (OpenAI, Anthropic, etc.)
  - model_name: string - Specific model to use
  - temperature: float - Creativity parameter
  - max_tokens: int - Maximum tokens for response
- **Action Mapping Rules**: array of objects - Rules for translating LLM output to robot actions
  - llm_output_pattern: string - Pattern to match in LLM output
  - robot_action: string - Corresponding robot action
  - validation_criteria: object - Criteria to validate the action

### Multimodal Input Processing Result
- **Processing ID**: string (required) - Unique identifier for the processing result
- **Visual Input**: object - Processed visual information
  - detected_objects: array of objects - Objects identified in the visual field
  - object_properties: object - Properties of detected objects (color, size, position)
  - scene_description: string - Natural language description of the scene
- **Audio Input**: object - Processed audio information
  - recognized_text: string - Speech-to-text result
  - confidence_score: float - Confidence in speech recognition
  - audio_properties: object - Properties of audio (volume, clarity, etc.)
- **Fused Understanding**: object - Combined understanding from multiple modalities
  - unified_context: string - Unified context from all inputs
  - confidence_score: float - Overall confidence in multimodal understanding
  - ambiguity_level: enum (low, medium, high) - Level of ambiguity in understanding

### Action Execution Result
- **Execution ID**: string (required) - Unique identifier for the execution
- **Action Sequence**: array of objects - The sequence of actions executed
  - action_type: string - Type of action executed
  - parameters: object - Parameters used for the action
  - start_time: datetime - When the action started
  - end_time: datetime - When the action completed
  - success_status: boolean - Whether the action was successful
  - error_message: string (optional) - Error message if action failed
- **Robot State Changes**: object - Changes to robot state during execution
  - position_change: object - Change in robot position
  - manipulation_result: object - Result of manipulation actions
  - sensor_feedback: object - Feedback from robot sensors
- **User Feedback**: object - Feedback provided to the user
  - status_message: string - Status message for the user
  - progress: float - Progress percentage (0-1)

## Relationships

- A **Voice Command** generates one or more **Cognitive Plans**
- A **Cognitive Plan** connects to **VLA Pipeline Configuration** for execution parameters
- **VLA Pipeline Configuration** uses **Multimodal Input Processing Result** as input
- **Multimodal Input Processing Result** connects to **Action Execution Result** for output
- **Action Execution Result** provides feedback to update **VLA Pipeline Configuration**

## Validation Rules

- Voice Command text must not be empty
- Cognitive Plan action sequences must contain at least one action
- LLM temperature must be between 0.0 and 1.0
- Action execution parameters must match the expected format for the action type
- Safety validation must pass before any physical action execution
- Confidence scores must be between 0.0 and 1.0
- Action sequences must not contain physically impossible combinations