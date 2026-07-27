// HUB75 Studio installer. Plain script, no build step.
(function () {
  'use strict';

  var REPO = 'pavlov-net/hub75-studio';

  // ---- theme -------------------------------------------------------------
  var toggle = document.getElementById('theme-toggle');

  function currentTheme() {
    var set = document.documentElement.getAttribute('data-theme');
    if (set) return set;
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  }

  if (toggle) {
    toggle.addEventListener('click', function () {
      var next = currentTheme() === 'dark' ? 'light' : 'dark';
      document.documentElement.setAttribute('data-theme', next);
      try {
        localStorage.setItem('theme', next);
      } catch (e) {}
    });
  }

  // ---- controller picker -------------------------------------------------
  var installButton = document.querySelector('esp-web-install-button');

  document.querySelectorAll('input[name="controller"]').forEach(function (radio) {
    radio.addEventListener('change', function () {
      if (!installButton || !radio.checked) return;
      installButton.setAttribute('manifest', 'firmware/' + radio.value + '.manifest.json');
    });
  });

  // ---- latest release ----------------------------------------------------
  // Decoration only: if GitHub is unreachable or rate limits us, the line stays
  // hidden and the installer works exactly the same.
  var line = document.getElementById('release-line');
  if (!line) return;

  fetch('https://api.github.com/repos/' + REPO + '/releases/latest', {
    headers: { Accept: 'application/vnd.github+json' }
  })
    .then(function (res) {
      if (!res.ok) throw new Error('HTTP ' + res.status);
      return res.json();
    })
    .then(function (release) {
      if (!release || !release.tag_name) return;

      var date = '';
      if (release.published_at) {
        var d = new Date(release.published_at);
        if (!isNaN(d)) {
          date = d.toLocaleDateString(undefined, { year: 'numeric', month: 'short', day: 'numeric' });
        }
      }

      // textContent throughout: the tag name is remote input and never markup.
      line.textContent = 'Installing ' + release.tag_name + (date ? ' · released ' + date + ' · ' : ' · ');

      var link = document.createElement('a');
      link.href = release.html_url || 'https://github.com/' + REPO + '/releases';
      link.textContent = 'release notes';
      line.appendChild(link);
      line.hidden = false;
    })
    .catch(function () {});
}());
