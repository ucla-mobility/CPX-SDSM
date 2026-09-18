/* Figure-mode toggle shared by every visualization page.
 *
 * Screen mode is the interactive default; figure mode thins the grid, thickens the
 * series and enlarges tick and axis labels so a crop of the canvas reads correctly
 * at conference-poster viewing distance.
 *
 * The pages draw their charts on raw canvas, so stroke weights cannot come from CSS.
 * They are defined once here and read at draw time via vizFigure(); a page never
 * decides what a mode looks like, it only asks. Adding a page costs a <link> and a
 * <script> — nothing in this file changes.
 *
 * Must load before the page's own chart script (plain <script> in <head>, no defer),
 * so vizFigure() is defined by the time a draw function first runs.
 */
(function () {
  var KEY = 'gtp-figure-mode';

  /* axisFont is deliberately heavier and larger than tickFont: an axis title and a
     tick number sharing one size is the main thing that reads as unfinished at
     poster distance. Pages ask for the role, never the size.

     Bigger type needs more gutter or the tick numbers collide with the rotated
     y-title, so the padding the typography requires travels with it: pages add
     padLBoost / padBBoost to their own base padding and read axisPad for the
     y-title baseline. Both boosts are 0 on screen, so screen layout is unchanged.

     insetFont is deliberately identical in both modes: fixed-size inset diagrams in
     the control panel are never part of a poster crop, and their labels clip if they
     grow inside a canvas whose pixel size does not. */
  var MODES = {
    screen: { series: 2.5, thin: 1.5, grid: 1, gridAlpha: 0.5, tickFont: '11px Cambria, Georgia, "Times New Roman", serif', smallFont: '10px Cambria, Georgia, "Times New Roman", serif', axisFont: '600 12px Cambria, Georgia, "Times New Roman", serif', insetFont: '10px Cambria, Georgia, "Times New Roman", serif', axisPad: 14, padLBoost: 0, padBBoost: 0 },
    figure: { series: 4.0, thin: 2.6, grid: 0.6, gridAlpha: 0.32, tickFont: '20px Cambria, Georgia, "Times New Roman", serif', smallFont: '20px Cambria, Georgia, "Times New Roman", serif', axisFont: '600 20px Cambria, Georgia, "Times New Roman", serif', insetFont: '10px Cambria, Georgia, "Times New Roman", serif', axisPad: 24, padLBoost: 22, padBBoost: 16 }
  };

  function enabled() {
    try { return localStorage.getItem(KEY) === '1'; } catch (e) { return false; }
  }

  /* The draw-time interface: pages read weights and fonts, never the mode itself. */
  window.vizFigure = function () {
    return enabled() ? MODES.figure : MODES.screen;
  };

  /* Draw a rotated y-axis title centred over the full canvas height, shrinking the font
     only if the label would otherwise overrun the (shorter, in figure mode) chart box.
     Pages pass the text, the left inset x, and the canvas height h, and set fillStyle /
     font before calling — the page asks for a fitted title, it never computes the fit.
     Uses the current ctx.font for measurement; save()/restore() leaves ctx untouched. */
  window.vizYTitle = function (ctx, text, x, h) {
    var span = h - 16;  // 8px breathing room top and bottom
    var w = ctx.measureText(text).width;
    ctx.save();
    if (w > span) {
      var m = /(\d+(?:\.\d+)?)px/.exec(ctx.font);
      if (m) {
        var fitted = Math.max(13, parseFloat(m[1]) * span / w);
        ctx.font = ctx.font.replace(/(\d+(?:\.\d+)?)px/, fitted.toFixed(1) + 'px');
      }
    }
    ctx.translate(x, h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText(text, 0, 0);
    ctx.restore();
  };

  function cssVar(name, fallback) {
    try {
      var v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
      return v || fallback;
    } catch (e) { return fallback; }
  }

  /* freshness_decay.html is the one page using Chart.js; the rest are hand-drawn
     canvas and repaint off the resize event below. */
  function restyleChartJs() {
    if (typeof Chart === 'undefined' || !Chart.instances) return;
    var f = window.vizFigure();
    var grid = enabled() ? cssVar('--border', 'rgba(0,0,0,0.28)') : cssVar('--border', 'rgba(0,0,0,0.12)');
    Object.keys(Chart.instances).forEach(function (id) {
      var c = Chart.instances[id];
      if (!c) return;
      /* Chart.js v4 exposes chart.options as a resolver proxy that is rebuilt from
         chart.config.options on every update(), so writes to chart.options are
         silently discarded. Style the real config; fall back for older versions. */
      var o = (c.config && c.config.options) || c.options;
      if (!o) return;
      o.font = o.font || {};
      o.font.size = enabled() ? 20 : 11;
      ['x', 'y'].forEach(function (axis) {
        var s = o.scales && o.scales[axis];
        if (!s) return;
        s.grid = s.grid || {};
        s.grid.color = grid;
        s.grid.lineWidth = f.grid;
        s.ticks = s.ticks || {};
        s.ticks.font = { size: enabled() ? 20 : 11 };
        s.title = s.title || {};
        s.title.font = { size: enabled() ? 20 : 12, weight: 600 };
      });
      c.data.datasets.forEach(function (d) {
        if (d.type === 'scatter' || d.showLine === false) return;
        d.borderWidth = d.borderDash ? f.thin : f.series;
      });
      c.update('none');
    });
  }

  function apply() {
    if (document.body) document.body.classList.toggle('figure-mode', enabled());
    var btn = document.querySelector('.viz-mode-toggle');
    if (btn) btn.setAttribute('aria-pressed', enabled() ? 'true' : 'false');
    /* Every hand-drawn page already redraws on resize and re-reads its CSS vars and
       vizFigure() as it does, so one synthetic event repaints them all. */
    window.dispatchEvent(new Event('resize'));
    restyleChartJs();
  }

  function mount() {
    var btn = document.createElement('button');
    btn.className = 'viz-mode-toggle';
    btn.type = 'button';
    btn.setAttribute('aria-pressed', 'false');
    btn.innerHTML = '<span class="box"></span>Figure mode';
    btn.addEventListener('click', function () {
      try { localStorage.setItem(KEY, enabled() ? '0' : '1'); } catch (e) {}
      apply();
    });
    document.body.appendChild(btn);
    apply();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', mount);
  } else {
    mount();
  }
})();
