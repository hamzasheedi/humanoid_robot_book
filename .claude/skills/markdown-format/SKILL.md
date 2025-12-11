---
name: format_markdown_skill
description: Ensures that all Markdown content generated for the Physical AI & Humanoid Robotics Textbook is consistently and correctly formatted, improving readability, compatibility with Docusaurus, and quality of generated documentation.
---

# Format Markdown Skill

## Overview

This skill formats and cleans up markdown content to ensure consistent styling, proper structure, and Docusaurus compatibility for the Physical AI & Humanoid Robotics Textbook.

## Input

```json
{
  "markdown": "Raw Markdown text (chapter content, notes, code snippets, tables, images, callouts, etc.)"
}
```

## Output

```json
{
  "formatted": "Cleaned and standardized Markdown text"
}
```

## Resources

This skill includes resource directories that demonstrate how to organize different types of bundled resources:

### scripts/
Executable code (Python/Bash/etc.) that can be run directly to perform markdown formatting operations.

### references/
Documentation and reference material for Markdown specifications, Docusaurus markdown requirements, and formatting guidelines.

### assets/
Template files and examples of properly formatted markdown content for reference and testing.

## Functionality

### Format Headings
- Ensure that all Markdown headings (#, ##, ###, etc.) are followed by a space
- `#Heading1` → `# Heading1`
- `##Subheading` → `## Subheading`
- Normalize heading levels for Docusaurus sidebar consistency

### Format Code Blocks
- Standardize fenced code blocks with triple backticks
- Trim leading/trailing whitespace inside code blocks
- Preserve language identifiers (e.g., ```js, ```python)
- Remove malformed code block sequences
- Example:
```
```js
console.log("hello world")
```
```

### Format Tables
- Trim extra whitespace in table columns
- Ensure proper | separation and row alignment
- Convert inconsistent tables to clean format suitable for Docusaurus
- Example:
```
| Name | Age |
|------|-----|
| John | 25  |
```

### Format Callouts / Notes
- Convert standard callouts to Docusaurus-supported callouts
- `:::note` → `:::info`
- Preserve `:::tip`, `:::warning`, `:::danger` callouts
- Ensure proper opening and closing of callout blocks

### Format Images
- Normalize image Markdown syntax
- Remove extra spaces in alt text and URL
- Ensure proper formatting: `![Alt text](url)`
- Convert local image paths to relative paths for Docusaurus static/ folder

### Miscellaneous Formatting
- Remove trailing spaces on lines
- Ensure consistent newline characters
- Validate Markdown syntax for Docusaurus compatibility
- Preserve all content while improving formatting consistency