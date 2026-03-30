# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
import subprocess
import time
import sys
import os
import yaml
import socket

def load_config_to_env(config_path):
    if not os.path.exists(config_path):
        print(f"Config not found at {config_path}")
        return

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        cs = config.get('content_search', {})

        # MinIO
        minio = cs.get('minio', {})
        os.environ["MINIO_SERVER"] = str(minio.get('server', "127.0.0.1:9000"))
        os.environ["MINIO_ROOT_USER"] = str(minio.get('root_user', "minioadmin"))
        os.environ["MINIO_ROOT_PASSWORD"] = str(minio.get('root_password', "minioadmin"))
        os.environ["MINIO_BUCKET"] = str(minio.get('bucket', "content-search"))
        os.environ["MINIO_SECURE"] = "False"

        # ChromaDB
        chroma = cs.get('chromadb', {})
        os.environ["CHROMA_HOST"] = str(chroma.get('host', "127.0.0.1"))
        os.environ["CHROMA_PORT"] = str(chroma.get('port', 9090))
        
        # File Ingest
        fi = cs.get('file_ingest', {})
        os.environ["FILE_INGEST_HOST"] = str(fi.get('host_addr', "127.0.0.1"))
        os.environ["FILE_INGEST_PORT"] = str(fi.get('port', 9990))
        os.environ["CHROMA_COLLECTION_NAME"] = str(fi.get('collection_name', "content-search"))
        os.environ["INGEST_DEVICE"] = str(fi.get('device', "CPU"))
        os.environ["VISUAL_EMBEDDING_MODEL"] = str(fi.get('visual_embedding_model', "CLIP/clip-vit-b-16"))
        os.environ["DOC_EMBEDDING_MODEL"] = str(fi.get('doc_embedding_model', "BAAI/bge-small-en-v1.5"))

        # Video Preprocess
        vp = cs.get('video_preprocess', {})
        os.environ["VIDEO_PREPROCESS_HOST"] = str(vp.get('host_addr', "127.0.0.1"))
        os.environ["VIDEO_PREPROCESS_PORT"] = str(vp.get('port', 8001))
        os.environ["CHUNK_DURATION_S"] = str(vp.get('chunk_duration_s', 30))
        os.environ["CHUNK_OVERLAP_S"] = str(vp.get('chunk_overlap_s', 4))
        os.environ["MAX_NUM_FRAMES"] = str(vp.get('max_num_frames', 8))
        os.environ["VLM_TIMEOUT_SECONDS"] = str(vp.get('vlm_timeout_seconds', 300))

        # VLM
        vlm = cs.get('vlm', {})
        os.environ["VLM_HOST"] = str(vlm.get('host_addr', "127.0.0.1"))
        os.environ["VLM_PORT"] = str(vlm.get('port', 9900))

        print(f"Config loaded from: {os.path.abspath(config_path)}")
        print("Environment variables injected.")

    except Exception as e:
        print(f"Error parsing config: {e}")

def is_port_open(host, port, timeout=1):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (ConnectionRefusedError, socket.timeout, OSError):
        return False

def wait_for_service(name, host, port, max_retries=20):
    print(f"  [?] Waiting for {name} on {host}:{port}...", end="", flush=True)
    for _ in range(max_retries):
        if is_port_open(host, port):
            print(" [READY]")
            return True
        print(".", end="", flush=True)
        time.sleep(1)
    print(" [FAILED]")
    return False

def launch(name, cmd):
    print(f"  [+] Launching {name}...")
    return subprocess.Popen(cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)

def start_dev_environment():
    cwd = os.getcwd()
    python_exe = sys.executable
    processes = []

    config_file = os.path.join(cwd, "..", "config.yaml")
    load_config_to_env(config_file)
    print(f"Starting Services with Dynamic Configuration...\n")
    try:
        # === STAGE 1: Infrastructure ===
        print("--- STAGE 1: Infrastructure ---")
        m_srv = os.getenv("MINIO_SERVER").split(':')
        processes.append(launch("MinIO", [
            os.path.abspath(r".\providers\minio_wrapper\minio.exe"), "server", 
            os.path.abspath(r".\providers\minio_wrapper\minio_data"), 
            "--address", os.getenv("MINIO_SERVER"), "--console-address", ":9001"
        ]))

        processes.append(launch("ChromaDB", [
            python_exe, "-m", "uvicorn", "chromadb.app:app", 
            "--host", os.getenv("CHROMA_HOST"), "--port", os.getenv("CHROMA_PORT")
        ]))

        wait_for_service("MinIO", m_srv[0], int(m_srv[1]))
        wait_for_service("ChromaDB", os.getenv("CHROMA_HOST"), int(os.getenv("CHROMA_PORT")))

        # === STAGE 2: Core Sub-services ===
        print("\n--- STAGE 2: Core Sub-services ---")
        processes.append(launch("File Ingest Service", [
            python_exe, "-m", "uvicorn", "providers.file_ingest_and_retrieve.server:app", 
            "--host", os.getenv("FILE_INGEST_HOST"), "--port", os.getenv("FILE_INGEST_PORT")
        ]))

        processes.append(launch("Video Preprocess Service", [
            python_exe, "-m", "uvicorn", "providers.video_preprocess.server:app", 
            "--host", os.getenv("VIDEO_PREPROCESS_HOST"), "--port", os.getenv("VIDEO_PREPROCESS_PORT")
        ]))

        wait_for_service("File Ingest", os.getenv("FILE_INGEST_HOST"), int(os.getenv("FILE_INGEST_PORT")), 100)
        wait_for_service("Video Preprocess", os.getenv("VIDEO_PREPROCESS_HOST"), int(os.getenv("VIDEO_PREPROCESS_PORT")), 60)

        # === STAGE 3: Main App ===
        print("\n--- STAGE 3: Main App ---")
        subprocess.run([python_exe, "main.py"])

    except KeyboardInterrupt:
        print("\n\nShutdown signal received.")
    finally:
        print("\nCleaning up background processes...")
        for proc in processes:
            try:
                subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)], 
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        print("All background windows closed.")

if __name__ == "__main__":
    start_dev_environment()