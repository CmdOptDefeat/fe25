import os
import datetime
import logging

def setup_logger():
    # Ensure logs/ directory exists
    log_dir = "initial-tests/logger/logs"
    os.makedirs(log_dir, exist_ok=True)

    # Create filename with date and time
    now = datetime.datetime.now()
    log_filename = f"log_{now.strftime('%Y-%m-%d_%H-%M-%S')}.log"
    log_path = os.path.join(log_dir, log_filename)

    # Configure logging to append mode
    logging.basicConfig(
        filename=log_path,
        filemode='a',  # Append mode
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )
    return log_path

if __name__ == "__main__":
    log_file = setup_logger()
    logging.info("Program started")
    logging.info("Example log message")
    logging.warning("This is a warning")
    logging.error("This is an error")
    print(f"Logging to file: {log_file}")
