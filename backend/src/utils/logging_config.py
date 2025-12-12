"""
Logging configuration for the Physical AI & Humanoid Robotics Textbook platform
Provides structured logging for authentication and personalization services
"""

import logging
import sys
from datetime import datetime
from enum import Enum
from typing import Optional, Dict, Any
import json
from pythonjsonlogger import jsonlogger

class LogLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"

class LogCategory(Enum):
    AUTHENTICATION = "authentication"
    PERSONALIZATION = "personalization"
    PROFILE = "profile"
    SESSION = "session"
    DATABASE = "database"
    API = "api"
    SECURITY = "security"

class LoggingConfig:
    """
    Configuration class for structured logging
    """

    def __init__(self, log_level: str = "INFO", log_format: str = "json"):
        self.log_level = getattr(logging, log_level.upper(), logging.INFO)
        self.log_format = log_format
        self.logger = None

    def setup_logging(self):
        """
        Set up the logging configuration
        """
        # Create logger
        self.logger = logging.getLogger("humanoid_robotics_auth")
        self.logger.setLevel(self.log_level)

        # Prevent adding handlers multiple times
        if self.logger.handlers:
            return self.logger

        # Create console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(self.log_level)

        # Create formatter based on format type
        if self.log_format == "json":
            formatter = jsonlogger.JsonFormatter(
                '%(timestamp)s %(level)s %(name)s %(message)s %(module)s %(function)s %(line)s',
                rename_fields={
                    'timestamp': '@timestamp',
                    'level': 'level',
                    'name': 'logger',
                    'module': 'module',
                    'function': 'function',
                    'line': 'line'
                }
            )
        else:
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s - [%(module)s:%(funcName)s:%(lineno)d]'
            )

        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        return self.logger

def log_event(
    category: LogCategory,
    level: LogLevel,
    message: str,
    user_id: Optional[str] = None,
    session_id: Optional[str] = None,
    additional_data: Optional[Dict[str, Any]] = None
):
    """
    Log an event with structured data
    """
    logger = LoggingConfig().setup_logging()

    log_data = {
        "category": category.value,
        "level": level.value,
        "message": message,
        "timestamp": datetime.utcnow().isoformat(),
        "user_id": user_id,
        "session_id": session_id,
    }

    if additional_data:
        log_data.update(additional_data)

    # Format and log the message
    log_message = json.dumps(log_data)

    getattr(logger, level.value.lower())(log_message)

def log_authentication_event(
    event_type: str,
    user_email: str,
    success: bool,
    user_id: Optional[str] = None,
    additional_data: Optional[Dict[str, Any]] = None
):
    """
    Log authentication-specific events
    """
    additional_data = additional_data or {}
    additional_data.update({
        "event_type": event_type,
        "user_email": user_email,
        "success": success
    })

    log_event(
        LogCategory.AUTHENTICATION,
        LogLevel.INFO if success else LogLevel.WARNING,
        f"Authentication {event_type} {'successful' if success else 'failed'} for {user_email}",
        user_id=user_id,
        additional_data=additional_data
    )

def log_personalization_event(
    event_type: str,
    user_id: str,
    success: bool,
    additional_data: Optional[Dict[str, Any]] = None
):
    """
    Log personalization-specific events
    """
    additional_data = additional_data or {}
    additional_data.update({
        "event_type": event_type,
        "success": success
    })

    log_event(
        LogCategory.PERSONALIZATION,
        LogLevel.INFO if success else LogLevel.WARNING,
        f"Personalization {event_type} {'successful' for success else 'failed'} for user {user_id}",
        user_id=user_id,
        additional_data=additional_data
    )

def log_security_event(
    event_type: str,
    user_id: Optional[str] = None,
    ip_address: Optional[str] = None,
    severity: LogLevel = LogLevel.WARNING,
    additional_data: Optional[Dict[str, Any]] = None
):
    """
    Log security-related events
    """
    additional_data = additional_data or {}
    additional_data.update({
        "event_type": event_type,
        "ip_address": ip_address
    })

    log_event(
        LogCategory.SECURITY,
        severity,
        f"Security event: {event_type}",
        user_id=user_id,
        additional_data=additional_data
    )

# Initialize the logger
logger = LoggingConfig().setup_logging()

# Example usage:
if __name__ == "__main__":
    # Test logging
    logger.info("Logging system initialized")

    # Test structured logging
    log_authentication_event("signup", "test@example.com", True, "user_123", {"os": "Windows", "cpu": "Intel"})
    log_personalization_event("update", "user_123", True, {"content_difficulty": "advanced"})
    log_security_event("failed_login", "user_123", "192.168.1.1", LogLevel.WARNING)