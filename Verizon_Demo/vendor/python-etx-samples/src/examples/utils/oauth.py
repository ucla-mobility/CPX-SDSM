# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
from dataclasses import dataclass
from examples.utils.config import AppConfig
import requests

@dataclass
class TokenPair:
    """
    Data container for the two-tier authentication tokens required by Verizon.

    Attributes:
        access_token: The OAuth2 bearer token for API authorization.
        session_token: The M2M session token for specific device operations.
    """
    access_token: str
    session_token: str

def get_thingspace_token(config: AppConfig) -> TokenPair:
    """
    Orchestrates the two-step authentication flow for Verizon ThingSpace.

    This function performs:
    1. A Client Credentials exchange for an OAuth2 Access Token (valid for 60 mins).
    2. A Session Login using that Access Token + User Credentials for a Session Token.
       This session token has a rolling 20 minutes validity period.

    Args:
        config: An instance of AppConfig containing endpoints and identity data.

    Returns:
        TokenPair: An object containing both valid tokens.

    Raises:
        requests.exceptions.RequestException: If any network call fails.
        KeyError: If the API response format deviates from the expected structure.
    """

    # --- STEP 1: OBTAIN OAUTH2 ACCESS TOKEN ---
    # We use the Base64 'token' (Client ID:Secret) from config.identity.token

    access_headers = {
        "Content-Type": "application/x-www-form-urlencoded",
        "Authorization": f"Basic {config.identity.token}"
    }

    # ThingSpace requires grant_type=client_credentials for this flow
    access_data = {
        "grant_type": "client_credentials",
    }

    # config.endpoints.oauthTimeout is in ms; requests.post expects seconds
    access_response = requests.post(
        config.endpoints.oauthUrl,
        headers=access_headers,
        data=access_data,
        timeout=config.endpoints.oauthTimeout / 1000
    )

    # Raise exception for 4xx or 5xx errors
    access_response.raise_for_status()

    # Extract the bearer token from the JSON response
    access_json = access_response.json()
    access_token = access_json["access_token"]

    # --- STEP 2: OBTAIN M2M SESSION TOKEN ---
    # We use the Access Token from Step 1 and the User/Password from config.identity

    session_headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {access_token}"
    }

    # Payload requires the ThingSpace portal username and password
    session_payload = {
        "username": config.identity.user,
        "password": config.identity.password,
    }

    session_response = requests.post(
        config.endpoints.sessionUrl,
        headers=session_headers,
        json=session_payload,
        timeout=config.endpoints.oauthTimeout / 1000
    )

    # Raise exception for 4xx or 5xx errors
    session_response.raise_for_status()

    # ThingSpace returns 'sessionToken' (camelCase).
    # This token is typically used in the 'VZ-M2M-Token' header for V2X APIs.
    session_json = session_response.json()
    session_token = session_json["sessionToken"]

    return TokenPair(
        access_token=access_token,
        session_token=session_token
    )