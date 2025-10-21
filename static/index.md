# About

Ready-to-use ESPHome firmware for HUB75 LED matrix displays with Home Assistant integration. Includes a rich collection of applications—Now Playing with album art, Team Tracker for live sports scores, Clock & Weather Dashboard, Visual Effects, interactive Pong, and more. Built on ESPHome with LVGL for smooth graphics, everything integrates seamlessly with Home Assistant and can be fully customized via YAML configuration.

# Installation

You can use the button below to install the pre-built firmware directly to your device via USB from the browser.

<p>Select your controller:</p>
<ul class="radios">
<li>
    <label><input type="radio" name="type" value="apollo-automation-m1-rev4" data-manifest="apollo-automation-m1-rev4.manifest.json" checked/> Apollo Automation M-1 Rev4</label>
</li>
<li>
    <label><input type="radio" name="type" value="apollo-automation-m1-rev6" data-manifest="apollo-automation-m1-rev6.manifest.json" /> Apollo Automation M-1 Rev6</label>
</li>
<li>
    <label><input type="radio" name="type" value="adafruit-matrix-portal-s3" data-manifest="adafruit-matrix-portal-s3.manifest.json" /> Adafruit Matrix Portal S3</label>
</li>
</ul>

<esp-web-install-button manifest="firmware/apollo-automation-m1-rev4.manifest.json"></esp-web-install-button>

<script>
    document.querySelectorAll('input[name="type"]').forEach(radio =>
    radio.addEventListener("change", () => {
        const button = document.querySelector('esp-web-install-button');
        button.manifest = `firmware/${radio.dataset.manifest}`;
    }
    ));
</script>
<script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>
