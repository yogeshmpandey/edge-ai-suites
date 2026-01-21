# Get Host IP Address
host_ip=$(ip route get 1 | awk '{print $7}')

export REGISTRY_URL=intel/
export TAG=latest

# Export current user and group IDs for container user
export USER_ID=$(id -u)
export USER_GROUP_ID=$(id -g)
export VIDEO_GROUP_ID=$(getent group video | awk -F: '{printf "%s\n", $3}')
export RENDER_GROUP_ID=$(getent group render | awk -F: '{printf "%s\n", $3}')

# Multimodal embedding serving
export EMBEDDING_MODEL_NAME=${EMBEDDING_MODEL_NAME:-"CLIP/clip-vit-b-32"}
export EMBEDDING_USE_OV=false
export EMBEDDING_DEVICE=${EMBEDDING_DEVICE:-CPU}
export OV_PERFORMANCE_MODE=${OV_PERFORMANCE_MODE:-LATENCY}

# If EMBEDDING_DEVICE is GPU, set EMBEDDING_USE_OV to true
if [ "$EMBEDDING_DEVICE" = "GPU" ]; then
    export EMBEDDING_USE_OV=true
fi

export EMBEDDING_SERVER_PORT=9777
export EMBEDDING_MODEL_NAME=${EMBEDDING_MODEL_NAME}

# Model path configuration
export EMBEDDING_OV_MODELS_DIR=${EMBEDDING_OV_MODELS_DIR:-"/app/ov_models"}

# Check if EMBEDDING_MODEL_NAME is supported
case "$EMBEDDING_MODEL_NAME" in
    "CLIP/clip-vit-b-16"|"CLIP/clip-vit-l-14"|"CLIP/clip-vit-b-32"|"CLIP/clip-vit-h-14")
        echo "Using CLIP model: $EMBEDDING_MODEL_NAME"
        ;;
    "CN-CLIP/cn-clip-vit-b-16"|"CN-CLIP/cn-clip-vit-l-14"|"CN-CLIP/cn-clip-vit-h-14")
        echo "Using CN-CLIP model: $EMBEDDING_MODEL_NAME (Chinese + English support)"
        ;;
    "SigLIP/siglip2-vit-b-16"|"SigLIP/siglip2-vit-l-16"|"SigLIP/siglip2-so400m-patch16-384")
        echo "Using SigLIP model: $EMBEDDING_MODEL_NAME"
        ;;
    "MobileCLIP/mobileclip_s0"|"MobileCLIP/mobileclip_s1"|"MobileCLIP/mobileclip_s2"|"MobileCLIP/mobileclip_b"|"MobileCLIP/mobileclip_blt")
        echo "Using MobileCLIP model: $EMBEDDING_MODEL_NAME"
        ;;
    "Blip2/blip2_transformers")
        echo "Using BLIP2 model: $EMBEDDING_MODEL_NAME"
        ;;
    *)
        echo -e "WARNING: Model '$EMBEDDING_MODEL_NAME' may not be supported."
        echo -e "See docs/user-guide/supported-models.md for the complete list of supported models."
        ;;
esac

# env for vdms-vector-db
export VDMS_VDB_HOST_PORT=55555
export VDMS_VDB_HOST=vdms-vector-db


echo "All required environment variables set successfully."
echo "EMBEDDING_MODEL_NAME set to: ${EMBEDDING_MODEL_NAME}"
echo "EMBEDDING_DEVICE set to: ${EMBEDDING_DEVICE}"
echo "EMBEDDING_USE_OV set to: ${EMBEDDING_USE_OV}"
echo "OV_PERFORMANCE_MODE set to: ${OV_PERFORMANCE_MODE}"
