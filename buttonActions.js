// buttonActions.js - Gestion des actions des boutons

// Connecter les boutons HTML
document.getElementById('rotatePortsche').addEventListener('click', function() {
    if (isAnimating) return; // Ignorer pendant l'animation
    
    objects.porsche.angle = (objects.porsche.angle + 45) % 360;
    draw();
});

document.getElementById('rotateGolf').addEventListener('click', function() {
    if (isAnimating) return; // Ignorer pendant l'animation
    
    objects.golf.angle = (objects.golf.angle + 45) % 360;
    draw();
});

document.getElementById('rotateCiterne').addEventListener('click', function() {
    if (isAnimating) return; // Ignorer pendant l'animation
    
    objects.citerne.angle = (objects.citerne.angle + 45) % 360;
    draw();
});

document.getElementById('rotatePac').addEventListener('click', function() {
    if (isAnimating) return; // Ignorer pendant l'animation
    
    objects.pac.angle = (objects.pac.angle + 45) % 360;
    draw();
});

// Bouton d'animation
document.getElementById('animateParking').addEventListener('click', function() {
    animateParking();
});