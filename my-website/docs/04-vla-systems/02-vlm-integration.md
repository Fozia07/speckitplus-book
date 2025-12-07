---
sidebar_position: 2
title: Integrating Vision (VLMs)
---

# Vision-Language Models (VLMs) in Robotics

Robots need to understand what they see, not just as pixels, but as semantic concepts. **Vision-Language Models (VLMs)** bridge this gap by mapping images and text to the same vector space.

## 1. The Power of CLIP

**CLIP (Contrastive Language-Image Pre-training)**, developed by OpenAI, is the foundational model for many VLA systems.

### How it Works
1.  **Dual Encoders:** It has an Image Encoder (ResNet or ViT) and a Text Encoder (Transformer).
2.  **Contrastive Learning:** It was trained on 400M pairs of (image, text) to maximize the cosine similarity between correct pairs and minimize it for incorrect ones.
3.  **Zero-Shot Capability:** You can give it an image and a list of text labels (e.g., "a robot", "a cup", "a red ball"), and it will tell you which label best matches the image, without ever being explicitly trained on those labels.

## 2. Hands-On: Zero-Shot Classification

We have provided a conceptual script in `code_examples/module-04-vla/scripts/clip_image_captioning.py` that demonstrates the logic of using a VLM.

### Code Walkthrough

```python
# Conceptual logic using pseudo-code for clarity
def clip_zero_shot_classification(image, labels):
    # 1. Preprocess Inputs
    image_tensor = preprocess(image)
    text_tokens = tokenizer(labels)

    # 2. Forward Pass
    image_features = model.encode_image(image_tensor)
    text_features = model.encode_text(text_tokens)

    # 3. Calculate Similarity (Dot Product)
    logits_per_image = image_features @ text_features.t()
    probs = logits_per_image.softmax(dim=-1)

    return probs
```

### Application in Robotics
*   **Open-Vocabulary Detection:** Instead of training a detector for just "cup," you can search for "blue ceramic mug with coffee."
*   **Semantic Navigation:** "Go to the fire extinguisher." The robot scans the room, scores frames against the text embedding of "fire extinguisher," and moves towards the highest score.

## 3. Beyond CLIP: Image Captioning & VQA
Newer models like **LLaVA** or **GPT-4o** can generate full captions or answer questions about images ("Is the door open?"). This provides a richer state description for the Planner (discussed in the next chapter).
