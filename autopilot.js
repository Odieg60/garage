// autopilot.js - Gestionnaire pour l'animation du stationnement automatique
// Dépend de vehicle-models.js qui doit être chargé avant ce fichier

/**
 * Gestionnaire principal pour l'autopilot
 */
class AutopilotManager {
    constructor() {
        this.vehicles = [];
        this.controllers = {};
        this.currentVehicleIndex = 0;
        this.isRunning = false;
        this.animationFrame = 0;
        this.waitCounter = 0;
    }

    // Initialiser le système
    init() {
        this.vehicles = [
            new VehicleModel("golf", objects.golf),
            new VehicleModel("porsche", objects.porsche)
        ];
        
        // Créer les contrôleurs pour chaque véhicule
        this.vehicles.forEach(vehicle => {
            this.controllers[vehicle.name] = new PurePursuitController(vehicle);
        });
        
        this.currentVehicleIndex = 0;
        this.isRunning = true;
        this.animationFrame = 0;
        this.waitCounter = 0;
    }

    // Démarrer l'animation pour le véhicule actuel
    startCurrentVehicle() {
        if (this.currentVehicleIndex >= this.vehicles.length) {
            this.isRunning = false;
            document.getElementById('animateParking').disabled = false;
            return false;
        }
        
        const vehicle = this.vehicles[this.currentVehicleIndex];
        const controller = this.controllers[vehicle.name];
        
        // Planifier le chemin
        const planner = new PathPlanner(
            { x: vehicle.obj.pos[0], y: vehicle.obj.pos[1], angle: vehicle.obj.angle, name: vehicle.name },
            { x: finalPositions[vehicle.name].pos[0], y: finalPositions[vehicle.name].pos[1], angle: finalPositions[vehicle.name].angle }
        );
        const path = planner.planPath();
        
        // Définir le chemin pour le contrôleur
        controller.setPath(path);
        
        return true;
    }

    // Mettre à jour l'animation
    update() {
        if (!this.isRunning) return false;
        
        // Si on est en attente
        if (this.waitCounter > 0) {
            this.waitCounter--;
            return true;
        }
        
        this.animationFrame++;
        
        // Si aucun véhicule n'est en cours, en démarrer un
        if (this.currentVehicleIndex >= this.vehicles.length) {
            this.isRunning = false;
            return false;
        }
        
        const vehicle = this.vehicles[this.currentVehicleIndex];
        const controller = this.controllers[vehicle.name];
        
        // Premier frame: démarrer le véhicule actuel
        if (this.animationFrame === 1) {
            this.startCurrentVehicle();
        }
        
        // Mettre à jour le contrôleur
        const control = controller.update();
        
        // Mettre à jour le véhicule
        vehicle.update(control.steeringAngle, control.velocity);
        
        // Si le véhicule a atteint sa destination
        if (controller.reachedDestination) {
            // Positionner exactement à la destination finale
            const finalPos = finalPositions[vehicle.name].pos;
            const finalAngle = finalPositions[vehicle.name].angle;
            
            vehicle.obj.pos[0] = finalPos[0];
            vehicle.obj.pos[1] = finalPos[1];
            vehicle.obj.angle = finalAngle;
            vehicle.velocity = 0;
            vehicle.steeringAngle = 0;
            
            // Attendre un peu avant de passer au véhicule suivant
            this.currentVehicleIndex++;
            this.animationFrame = 0;
            this.waitCounter = 50; // Attendre 50 frames (environ 1 seconde)
            
            // Si c'était le dernier véhicule
            if (this.currentVehicleIndex >= this.vehicles.length) {
                this.isRunning = false;
                document.getElementById('animateParking').disabled = false;
                return false;
            }
        }
        
        return true;
    }

    // Dessiner les informations de debug
    drawDebug(ctx) {
        if (!this.isRunning || this.currentVehicleIndex >= this.vehicles.length) return;
        
        const vehicle = this.vehicles[this.currentVehicleIndex];
        const controller = this.controllers[vehicle.name];
        const path = controller.path;
        
        if (!path || path.length === 0) return;
        
        // Dessiner le chemin complet
        ctx.strokeStyle = "#FF0000";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(path[0].x, path[0].y);
        
        for (let i = 1; i < path.length; i++) {
            ctx.lineTo(path[i].x, path[i].y);
        }
        
        ctx.stroke();
        
        // Dessiner les points du chemin
        for (let i = 0; i < path.length; i++) {
            // Waypoints déjà atteints en gris
            if (i < controller.currentTargetIndex) {
                ctx.fillStyle = "#888888";
            } 
            // Waypoint actuel en vert
            else if (i === controller.currentTargetIndex) {
                ctx.fillStyle = "#00FF00";
            } 
            // Waypoints à venir en rouge
            else {
                ctx.fillStyle = "#FF0000";
            }
            
            ctx.beginPath();
            ctx.arc(path[i].x, path[i].y, 5, 0, 2 * Math.PI);
            ctx.fill();
            
            // Pour le waypoint actuel, dessiner le cercle de tolérance
            if (i === controller.currentTargetIndex) {
                ctx.strokeStyle = "#00FF00";
                ctx.lineWidth = 1;
                ctx.beginPath();
                
                // Tolérance différente selon que c'est le dernier point ou non
                const tolerance = (i === path.length - 1) 
                    ? controller.finalTolerance 
                    : controller.waypointTolerance;
                
                ctx.arc(path[i].x, path[i].y, tolerance, 0, 2 * Math.PI);
                ctx.stroke();
            }
        }
        
        // Dessiner le point lookahead
        const targetPoint = controller.findLookaheadPoint();
        if (targetPoint) {
            ctx.fillStyle = "#0000FF";
            ctx.beginPath();
            ctx.arc(targetPoint.x, targetPoint.y, 4, 0, 2 * Math.PI);
            ctx.fill();
            
            // Dessiner une ligne de la voiture au point lookahead
            ctx.strokeStyle = "#0000FF";
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(vehicle.obj.pos[0], vehicle.obj.pos[1]);
            ctx.lineTo(targetPoint.x, targetPoint.y);
            ctx.stroke();
        }
        
        // Dessiner l'angle de braquage et la direction du véhicule
        const headingRad = degToRad(vehicle.obj.angle);
        
        // MODIFIÉ: Ajout d'une flèche pour indiquer l'avant du véhicule (vers la gauche)
        // Dessiner une flèche bleue pointant vers la gauche (avant) du véhicule
        const leftHeadingRad = degToRad(vehicle.obj.angle + 180);
        ctx.strokeStyle = "#00AAFF"; // Bleu clair pour l'avant
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(vehicle.obj.pos[0], vehicle.obj.pos[1]);
        ctx.lineTo(
            vehicle.obj.pos[0] + Math.cos(leftHeadingRad) * 25,
            vehicle.obj.pos[1] + Math.sin(leftHeadingRad) * 25
        );
        ctx.stroke();
        
        // Dessiner une pointe de flèche pour l'avant
        const arrowSize = 8;
        ctx.beginPath();
        ctx.moveTo(
            vehicle.obj.pos[0] + Math.cos(leftHeadingRad) * 25,
            vehicle.obj.pos[1] + Math.sin(leftHeadingRad) * 25
        );
        ctx.lineTo(
            vehicle.obj.pos[0] + Math.cos(leftHeadingRad + 0.5) * 20,
            vehicle.obj.pos[1] + Math.sin(leftHeadingRad + 0.5) * 20
        );
        ctx.lineTo(
            vehicle.obj.pos[0] + Math.cos(leftHeadingRad - 0.5) * 20,
            vehicle.obj.pos[1] + Math.sin(leftHeadingRad - 0.5) * 20
        );
        ctx.closePath();
        ctx.fillStyle = "#00AAFF";
        ctx.fill();
        
        // Dessiner une flèche rouge pointant vers l'arrière (droite) du véhicule
        ctx.strokeStyle = "#FF5555"; // Rouge clair pour l'arrière
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(vehicle.obj.pos[0], vehicle.obj.pos[1]);
        ctx.lineTo(
            vehicle.obj.pos[0] + Math.cos(headingRad) * 15,
            vehicle.obj.pos[1] + Math.sin(headingRad) * 15
        );
        ctx.stroke();
        
        // Dessiner l'angle de braquage - MODIFIÉ: déplacer le point de départ
        // Maintenant on place l'indicateur de braquage du côté gauche (avant)
        const frontX = vehicle.obj.pos[0] + Math.cos(leftHeadingRad) * (vehicle.wheelbase/2);
        const frontY = vehicle.obj.pos[1] + Math.sin(leftHeadingRad) * (vehicle.wheelbase/2);
        
        const steeringRad = degToRad(vehicle.obj.angle + vehicle.steeringAngle);
        ctx.strokeStyle = controller.shouldReverseDirection ? "#FF00FF" : "#00FFFF"; // Magenta si marche arrière
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(frontX, frontY);
        ctx.lineTo(
            frontX + Math.cos(steeringRad) * 20,
            frontY + Math.sin(steeringRad) * 20
        );
        ctx.stroke();
        
        // Afficher les informations textuelles
        ctx.fillStyle = "#000000";
        ctx.font = "12px Arial";
        ctx.fillText(`Véhicule: ${vehicle.name}`, 20, 20);
        ctx.fillText(`Waypoint: ${controller.currentTargetIndex}/${path.length-1}`, 20, 40);
        ctx.fillText(`Vitesse: ${vehicle.velocity.toFixed(2)}`, 20, 60);
        ctx.fillText(`Direction: ${vehicle.steeringAngle.toFixed(2)}°`, 20, 80);
        ctx.fillText(`Distance parcourue: ${Math.round(vehicle.totalMovement)}px`, 20, 100);
        ctx.fillText(`Marche arrière: ${controller.shouldReverseDirection ? "Oui" : "Non"}`, 20, 120);
        ctx.fillText(`Tentatives: ${controller.stuckCount}/${controller.maxStuckCount}`, 20, 140);
        ctx.fillText(`Avant du véhicule: Orienté vers la GAUCHE`, 20, 160);
    }
}

// Instance globale du gestionnaire d'autopilot
const autopilot = new AutopilotManager();

// Fonction pour convertir degrés en radians (pour compatibilité avec vehicle-models.js)
function degToRad(degrees) {
    return degrees * Math.PI / 180;
}

// Fonction pour lancer l'animation de stationnement
function animateParking() {
    if (!isAnimating) {
        // Initialiser l'animation
        isAnimating = true;
        document.getElementById('animateParking').disabled = true;
        
        // Initialiser et démarrer l'autopilot
        autopilot.init();
        
        // Lancer la boucle d'animation
        animationLoop();
        
        // Démarrer le rendu continu
        requestAnimationFrame(draw);
    }
}

// Boucle principale d'animation
function animationLoop() {
    if (!isAnimating) return;
    
    // Mettre à jour l'autopilot
    const continueAnimation = autopilot.update();
    
    // Si l'animation doit continuer
    if (continueAnimation) {
        setTimeout(animationLoop, 20); // ~50fps
    } else {
        isAnimating = false;
    }
}

// Fonction pour afficher les informations de debug
function drawAutopilotDebug(ctx) {
    if (isAnimating) {
        autopilot.drawDebug(ctx);
    }
}
