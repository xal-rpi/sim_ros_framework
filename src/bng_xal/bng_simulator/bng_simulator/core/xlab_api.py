"""
Python API wrapper for xlab Lua extension.

This module provides Python functions to interact with the xlab/xlabCore Lua extension
loaded in BeamNG. The xlab extension MUST be loaded for these functions to work.

Usage:
    beamng = BeamNGpy(host='127.0.0.1', port=25252)
    beamng.open(extensions=['xlab/xlabCore'])
    
    level = XlabApi.get_current_level(beamng)
    XlabApi.start_level(beamng, 'derby')
"""

from typing import Optional, Dict, Any
from beamngpy import BeamNGpy


class XlabApi:
    """
    Wrapper for xlab extension API calls.
    
    All methods require xlab/xlabCore extension to be loaded in BeamNG.
    """
    
    @staticmethod
    def is_available(beamng: BeamNGpy) -> bool:
        """
        Check if xlab extension is loaded and available.
        
        Args:
            beamng: Active BeamNGpy connection
            
        Returns:
            True if xlab extension is available, False otherwise
        """
        try:
            # Test with GetCurrentLevel call
            data = {"type": "GetCurrentLevel"}
            resp = beamng._send(data).recv("GetCurrentLevel")
            return True
        except Exception:
            return False
    
    @staticmethod
    def get_current_level(beamng: BeamNGpy) -> Optional[str]:
        """
        Get the name of the currently loaded level.
        
        Args:
            beamng: Active BeamNGpy connection
            
        Returns:
            Level name (e.g., "derby", "tech_ground") or None if no level loaded
            
        Raises:
            RuntimeError: If xlab extension is not loaded
        """
        data = {"type": "GetCurrentLevel"}
        resp = beamng._send(data).recv("GetCurrentLevel")
        
        if not resp.get("loaded", False):
            return None
        
        return resp.get("level")
    
    @staticmethod
    def start_level(beamng: BeamNGpy, level_name: str):
        """
        Load a level in BeamNG.
        
        Args:
            beamng: Active BeamNGpy connection
            level_name: Name of level to load (e.g., "derby", "tech_ground", "west_coast_usa")
            
        Raises:
            RuntimeError: If xlab extension is not loaded or level not found
        """
        data = {"type": "StartLevel", "levelName": level_name}
        beamng._send(data).ack("StartedLevel")
    
    @staticmethod
    def get_advanced_level_info(beamng: BeamNGpy, level_name: str) -> Dict[str, Any]:
        """
        Get detailed information about a level including spawn points and scenarios.
        
        Args:
            beamng: Active BeamNGpy connection
            level_name: Name of level
            
        Returns:
            Dict with keys:
                - levelName: str
                - levelInfo: Dict (full level metadata)
                - scenarios: List[Dict] (available scenarios for this level)
                - spawnPoints: List[Dict] (spawn points with pos/rot)
                
        Raises:
            RuntimeError: If xlab extension is not loaded or level not found
        """
        data = {"type": "GetAdvancedLevelInfo", "levelName": level_name}
        resp = beamng._send(data).recv("GetAdvancedLevelInfo")
        return resp
