# AFIF Smart Tank — Privacy Policy

_Last updated: 28 September 2026_

AFIF Smart Tank ("the app") is published by AFIF-TECH IoT Solutions (Afif Khoury, Jordan). It is the companion app for the AFIF-TECH water-tank controller. This policy explains what information the app handles.

## Summary
- No account, no sign-up, no advertising, no analytics or tracking.
- The app does not collect your name, contacts, location, photos or files.
- Tank data and your settings go only between your phone and **your** tank controller.

## Information the app handles

**Tank data and commands.** The app exchanges the tank level, pump state, alarms, rules and settings with your tank controller through an MQTT message broker (by default the public broker `broker.emqx.io`, operated by EMQ Technologies). This data describes your water tank, not you. Messages on the default public broker are **not encrypted** and could be read by anyone who knows your tank's topic name; you can choose an encrypted connection or your own private broker in the app's settings.

**Telegram usernames (optional).** If you turn on Telegram alerts, the Telegram usernames you enter are sent to your tank controller and stored there. The controller sends them to the CallMeBot service (callmebot.com) to deliver your alert messages and calls. Nothing is sent if you do not use this feature.

**App instance identifier.** The app creates a random identifier so the broker can tell your phone's connection apart from others. It is not linked to your identity.

**Settings and logs on your phone.** Your settings and a connection log are stored only on your phone. The log leaves the phone only if you choose "Share log".

**Firmware updates.** The app downloads firmware information from AFIF-TECH's public GitHub repository (github.com). GitHub may log standard request information such as the IP address, as any web server does.

## Permissions
- **Notifications and full-screen alerts** — to show the critical-low alarm and tank events.
- **Background service** — to keep the connection to your tank open so alarms arrive when the app is closed.
- **Nearby Wi-Fi devices / network access** — to set up and update the controller on your local WiFi. The app does not use your location.

## Children
The app is not directed at children under 13.

## Deleting your data
Uninstalling the app removes all its data from your phone. Telegram usernames can be removed in Settings → Telegram alerts (or by clearing the controller's settings).

## Changes
Changes to this policy will be published at this address with a new date.

## Contact
AFIF-TECH IoT Solutions — Afif Khoury, Jordan
WhatsApp: +962 79 714 5155
