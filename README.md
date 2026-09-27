# AFIF-TECH Smart Tank — firmware

Edit the sketch, raise `FW_VERSION`, commit → GitHub compiles, **signs** and publishes a Release → the **AFIF Smart Tank** app notifies you → one tap updates the tank over the internet.

## One-time setup (about 10 minutes)

1. **Create the repository** on github.com: *New repository* → name `AFIF-TECH-Firmware` → **Public** → *Create*.
   (It must be public so the app and the tank can download releases without a login. That is safe: the firmware is signed, see below.)
2. **Upload these files**: *Add file → Upload files* → drag in the `WaterTank_D1Mini` folder, `.github` folder, `.gitignore` and this README → *Commit changes*.
   If the `.github` folder does not upload, create `.github/workflows/release.yml` with *Add file → Create new file* and paste its content.
3. **Add the signing key as a secret**: *Settings → Secrets and variables → Actions → New repository secret*
   - Name: `SIGNING_KEY`
   - Secret: the **entire content** of `SIGNING_KEY-private-KEEP-SECRET.txt` (from `-----BEGIN` to `END ... KEY-----`).
   Then keep that file only in a safe offline place (USB stick / password manager) and delete other copies.
4. **Run the build**: *Actions* tab → *Build & publish firmware* → *Run workflow*. After ~4 minutes a Release **v2.2.0** appears with `WaterTank_D1Mini.ino.bin` and `version.json`.
   If it fails with "Resource not accessible by integration": *Settings → Actions → General → Workflow permissions → Read and write permissions* → Save, and run again.
5. **In the app**: *Firmware → Automatic updates from GitHub* → enter `afifkhoury2019/AFIF-TECH-Firmware` → *Check GitHub now*.

## Publishing a new firmware

1. Open `WaterTank_D1Mini/WaterTank_D1Mini.ino` on GitHub → pencil icon (edit).
2. Make your change **and raise** `#define FW_VERSION "2.2.0"` → e.g. `"2.2.1"`.
3. *Commit changes* — the commit message becomes the release notes shown in the app.
4. ~4 minutes later the release is published; within 6 hours every app notifies (or tap *Check GitHub now*). Tap **Update**.

The same version cannot be published twice — the build stops with a clear message if you forget to raise `FW_VERSION`.

## Why it is safe to be public: signed firmware

`WaterTank_D1Mini/public.key` makes the ESP8266 refuse any firmware that is not signed with the private key stored in the `SIGNING_KEY` secret — over the internet, over WiFi or through the `/update` page. Anyone can read this repository, but only your GitHub Actions can produce firmware your tanks will accept.

- **Keep a backup of the private key.** If it is lost, new firmware can only be installed by USB.
- **Never commit `private.key`** (it is in `.gitignore`).
- Tanks running firmware **2.1.0** accept the first signed update once (the app asks for the OTA password one time). From 2.2.0 on, only signed firmware is accepted.
- The OTA password in the sketch is now only the login of the local `/update` page; you may change it.

## Build settings (for reference)
Board LOLIN(WEMOS) D1 R2 & mini (`esp8266:esp8266:d1_mini`), ESP8266 core 3.1.2, default flash layout 4MB (FS:2MB OTA:~1019KB), libraries PubSubClient 2.8 and U8g2.
