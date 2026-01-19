#!/usr/bin/env python3
import sys
import os
import logging

# Ensure we can import from task_management
# We assume this script is run from the workspace root or src/task_management/scripts
# We need to add src/task_management to python path if not installed
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

try:
    from task_management.allocation_strategy import ILPAllocator
except ImportError:
    # Try finding it relative to this script if run directly
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    from task_management.allocation_strategy import ILPAllocator

def main():
    # Setup basic logging to stdout
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logger = logging.getLogger("TestAllocator")

    logger.info("Starting Standalone ILP Allocator Test...")

    try:
        allocator = ILPAllocator(logger)
        
        if allocator.use_ilp:
            logger.info("✅ ILP Allocator initialized successfully.")
            logger.info(f"Rules Path: {allocator.rules_path}")
        else:
            logger.warn("⚠️ ILP Allocator initialized but use_ilp is False. Fallback active.")
            
    except Exception as e:
        logger.error(f"❌ Failed to instantiate ILPAllocator: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
