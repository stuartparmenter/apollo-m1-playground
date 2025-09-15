# About

ESPHome based firmware for the Apollo Automation M‑1 HUB75 controller.  Contains some basic screens and lets you adopt it in ESPHome Builder in Home Assistant for additional pages, customization, etc.

# Installation

You can use the button below to install the pre-built firmware directly to your device via USB from the browser.

<p>Select your controller revision:</p>
<ul class="radios">
<li>
    <label><input type="radio" name="type" value="rev4" checked/> Rev 4</label>
</li>
<li>
    <label><input type="radio" name="type" value="rev6" /> Rev 6</label>
</li>
</ul>

<esp-web-install-button manifest="firmware/apollo-m1-rev4.manifest.json"></esp-web-install-button>

<script>
    document.querySelectorAll('input[name="type"]').forEach(radio =>
    radio.addEventListener("change", () => {
        const button = document.querySelector('esp-web-install-button');
        button.manifest = `firmware/apollo-m1-${radio.value}.manifest.json`;
    }
    ));
</script>
<script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>
