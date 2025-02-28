// vehicle-models.js - Modèle de véhicule et contrôleur pour l'autopilot

// Fonctions utilitaires
function degToRad(degrees) {
    return degrees * Math.PI / 180;
}

function radToDeg(radians) {
    return radians * 180 / Math.PI;
}

function distance(x1, y1, x2, y2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
}

function normalizeAngle(angle) {
    angle = angle % 360;
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

function shortestAngle(start, end) {
    let diff = (end - start) % 360;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

function lerp(start, end, t) {
    return start + (end - start) * t;
}

/**
 * Modèle de véhicule avec contraintes cinématiques
 */
class VehicleModel {
    constructor(name, obj, wheelbase) {
        this.name = name;
        this.obj = obj;
        this.wheelbase = wheelbase || (obj.width * 0.6);
        this.maxSteeringAngle = 30; // degrés
        this.steeringAngle = 0;
        this.velocity = 0;
        this.maxVelocity = 4;
        this.maxAcceleration = 0.2;
        this.maxDeceleration = 0.4;
        this.totalMovement = 0;  // Distance totale parcourue depuis le dernier waypoint
        this.lastPos = [obj.pos[0], obj.pos[1]]; // Dernière position pour le calcul du mouvement
    }

    // Mettre à jour le modèle de véhicule
    update(targetSteeringAngle, targetVelocity, dt = 1) {
        // Ajuster l'angle de braquage progressivement
        const steeringDelta = 2 * dt; // degrés par frame
        if (this.steeringAngle < targetSteeringAngle) {
            this.steeringAngle = Math.min(this.steeringAngle + steeringDelta, targetSteeringAngle);
        } else if (this.steeringAngle > targetSteeringAngle) {
            this.steeringAngle = Math.max(this.steeringAngle - steeringDelta, targetSteeringAngle);
        }
        this.steeringAngle = Math.max(-this.maxSteeringAngle, Math.min(this.maxSteeringAngle, this.steeringAngle));

        // Ajuster la vitesse progressivement
        if (this.velocity < targetVelocity) {
            this.velocity = Math.min(this.velocity + this.maxAcceleration * dt, targetVelocity);
        } else if (this.velocity > targetVelocity) {
            this.velocity = Math.max(this.velocity - this.maxDeceleration * dt, targetVelocity);
        }
        this.velocity = Math.max(-this.maxVelocity, Math.min(this.maxVelocity, this.velocity));

        // Sauvegarder la position actuelle pour calculer le mouvement
        const currentPos = [...this.obj.pos];

        // Appliquer le modèle bicycle pour le mouvement
        this.move(dt);

        // Calculer la distance parcourue
        const distanceMoved = distance(currentPos[0], currentPos[1], this.obj.pos[0], this.obj.pos[1]);
        this.totalMovement += distanceMoved;

        // Mettre à jour la dernière position
        this.lastPos = [...this.obj.pos];
    }

    // Déplacer le véhicule selon son état actuel
    move(dt = 1) {
        const headingRad = degToRad(this.obj.angle);
        const steeringRad = degToRad(this.steeringAngle);

        // Appliquer le modèle cinématique (bicycle model)
        if (Math.abs(this.steeringAngle) < 0.5) {
            // En ligne droite (éviter la division par presque zéro)
            this.obj.pos[0] += this.velocity * Math.cos(headingRad) * dt;
            this.obj.pos[1] += this.velocity * Math.sin(headingRad) * dt;
        } else {
            // En virage
            const turnRadius = this.wheelbase / Math.tan(steeringRad);
            const angularVelocity = this.velocity / turnRadius;
            const yawDelta = angularVelocity * dt;
            const newHeadingRad = headingRad + yawDelta;

            // Calculer le déplacement
            this.obj.pos[0] += turnRadius * (Math.sin(newHeadingRad) - Math.sin(headingRad));
            this.obj.pos[1] += turnRadius * (Math.cos(headingRad) - Math.cos(newHeadingRad));

            // Mettre à jour l'angle
            this.obj.angle = normalizeAngle(radToDeg(newHeadingRad));
        }
    }

    // Réinitialiser le compteur de mouvement
    resetMovementCounter() {
        this.totalMovement = 0;
        this.lastPos = [...this.obj.pos];
    }
}

/**
 * Contrôleur "Pure Pursuit" avec limite de mouvement et inversion
 * MODIFIÉ: L'avant du véhicule est maintenant orienté vers la gauche (180 degrés)
 */
class PurePursuitController {
    constructor(vehicle, lookaheadDistance = 30) {
        this.vehicle = vehicle;
        this.lookaheadDistance = lookaheadDistance;
        this.path = [];
        this.currentTargetIndex = 0;
        this.reachedDestination = false;
        
        // Paramètres de tolérance
        this.waypointTolerance = 35;      // Tolérance pour les waypoints intermédiaires
        this.angleTolerance = 25;         // Tolérance d'angle
        this.finalTolerance = 15;         // Tolérance pour le point final
        this.slowdownDistance = 100;      // Distance pour commencer à ralentir
        this.axisAlignmentAngle = 20;     // Angle max pour être considéré "dans l'axe"
        this.axisAlignmentDistance = 80;  // Distance pour vérifier l'alignement d'axe
        
        // Paramètres pour la limite de mouvement
        this.maxMovementBeforeRethink = 150; // Distance max avant de changer de stratégie
        this.shouldReverseDirection = false; // Drapeau pour inverser la direction
        this.stuckCount = 0;               // Compteur de tentatives échouées
        this.maxStuckCount = 3;            // Nombre max de tentatives avant de passer au waypoint suivant
        
        // Compteur pour éviter les changements trop rapides de waypoint
        this.waypointChangeDelay = 30;     // Frames à attendre avant de pouvoir changer de waypoint
        this.waypointChangeTimer = 0;      // Temps écoulé depuis le dernier changement
        
        // Angle pour déterminer si un waypoint est devant ou derrière (±90 degrés)
        this.frontSectorAngle = 90;
    }

    // Définir le chemin à suivre
    setPath(path) {
        this.path = path;
        this.currentTargetIndex = 0;
        this.reachedDestination = false;
        this.shouldReverseDirection = false;
        this.stuckCount = 0;
        this.waypointChangeTimer = 0;
        this.vehicle.resetMovementCounter();
    }

    // Mettre à jour le contrôleur
    update() {
        if (this.reachedDestination || this.path.length === 0) {
            return { steeringAngle: 0, velocity: 0 };
        }

        // Incrémenter le compteur pour les changements de waypoint
        this.waypointChangeTimer++;

        // Vérifier si le véhicule a trop bougé sans atteindre le waypoint
        this.checkMovementLimit();
        
        // Vérifier si le waypoint actuel a été atteint
        this.checkWaypointReached();
        
        // Vérifier si nous sommes dans l'axe pour sauter des waypoints intermédiaires
        // seulement si le délai minimum est passé
        if (this.waypointChangeTimer >= this.waypointChangeDelay) {
            this.checkAxisAlignment();
        }

        // Trouver le point cible sur le chemin
        const targetPoint = this.findLookaheadPoint();
        if (!targetPoint) {
            return { steeringAngle: 0, velocity: 0 };
        }

        // Calculer l'angle de braquage nécessaire
        const steeringAngle = this.calculateSteeringAngle(targetPoint);

        // Calculer la vitesse en fonction de la courbure et de la distance
        let velocity = this.calculateVelocity(steeringAngle, targetPoint);
        
        // Déterminer si le waypoint est devant ou derrière la voiture
        // MODIFIÉ: Maintenant l'avant du véhicule est vers la gauche (180 degrés)
        const isWaypointInFront = this.isPointInFrontSector(targetPoint);
        
        // Si le waypoint est devant, on avance; sinon, on recule
        // Sauf si on a forcé l'inversion de direction
        if (this.shouldReverseDirection) {
            velocity = -Math.abs(velocity) * 0.7; // Marche arrière forcée (plus lente)
        } else if (!isWaypointInFront) {
            velocity = -Math.abs(velocity) * 0.7; // Marche arrière (plus lente)
        } else {
            velocity = Math.abs(velocity); // Marche avant
        }

        return { steeringAngle, velocity };
    }
    
    // Vérifier si un point est dans le secteur avant de la voiture
    // MODIFIÉ: L'avant est maintenant vers la gauche (180 degrés d'offset)
    isPointInFrontSector(point) {
        const pos = this.vehicle.obj.pos;
        const heading = this.vehicle.obj.angle;
        
        // Calculer l'angle vers le point
        const pointAngle = radToDeg(Math.atan2(
            point.y - pos[1],
            point.x - pos[0]
        ));
        
        // Calculer la différence d'angle
        // MODIFIÉ: Ajout d'un offset de 180 degrés pour que l'avant soit vers la gauche
        const headingLeftFacing = normalizeAngle(heading + 180);
        const angleDiff = Math.abs(normalizeAngle(headingLeftFacing - pointAngle));
        
        // Si l'angle est inférieur à ±90 degrés, le point est devant (selon la nouvelle orientation)
        return angleDiff <= this.frontSectorAngle;
    }
    
    // Vérifier si le véhicule a trop bougé sans atteindre le waypoint
    checkMovementLimit() {
        // Ne pas vérifier pour le dernier waypoint
        if (this.currentTargetIndex >= this.path.length - 1) return;
        
        // Si le véhicule a trop bougé sans atteindre le waypoint
        if (this.vehicle.totalMovement > this.maxMovementBeforeRethink) {
            console.log(`${this.vehicle.name}: Mouvement excessif (${Math.round(this.vehicle.totalMovement)}px), changement de stratégie`);
            
            // Incrémenter le compteur de tentatives échouées
            this.stuckCount++;
            
            // Si on a trop essayé, passer au waypoint suivant
            if (this.stuckCount >= this.maxStuckCount) {
                console.log(`${this.vehicle.name}: Abandonnant le waypoint ${this.currentTargetIndex} après ${this.stuckCount} tentatives`);
                this.currentTargetIndex++;
                this.stuckCount = 0;
                this.shouldReverseDirection = false;
                this.waypointChangeTimer = 0; // Réinitialiser le timer de changement
            } else {
                // Sinon, inverser la direction
                this.shouldReverseDirection = !this.shouldReverseDirection;
            }
            
            // Réinitialiser le compteur de mouvement
            this.vehicle.resetMovementCounter();
        }
    }
    
    // Vérifier si le waypoint actuel a été atteint
    checkWaypointReached() {
        if (this.currentTargetIndex >= this.path.length) return;
        
        const pos = this.vehicle.obj.pos;
        const currentWaypoint = this.path[this.currentTargetIndex];
        const dist = distance(pos[0], pos[1], currentWaypoint.x, currentWaypoint.y);
        
        // Tolérance plus stricte pour le dernier waypoint
        const tolerance = (this.currentTargetIndex === this.path.length - 1) 
            ? this.finalTolerance : this.waypointTolerance;
        
        // Si on est assez proche du waypoint
        if (dist < tolerance) {
            // Vérifier aussi l'angle pour le dernier waypoint
            if (this.currentTargetIndex === this.path.length - 1) {
                const angleDiff = Math.abs(normalizeAngle(this.vehicle.obj.angle - currentWaypoint.angle));
                if (angleDiff <= this.angleTolerance) {
                    this.currentTargetIndex++;
                    
                    // Si c'était le dernier waypoint
                    if (this.currentTargetIndex >= this.path.length) {
                        this.reachedDestination = true;
                    }
                }
            } else {
                // Pour les waypoints intermédiaires, on passe au suivant sans vérifier l'angle
                this.currentTargetIndex++;
                this.shouldReverseDirection = false; // Réinitialiser l'inversion de direction
                this.stuckCount = 0;                // Réinitialiser le compteur de tentatives
                this.vehicle.resetMovementCounter(); // Réinitialiser le compteur de mouvement
                this.waypointChangeTimer = 0; // Réinitialiser le timer de changement de waypoint
            }
        }
    }
    
    // Vérifier si le véhicule est dans l'axe pour sauter des waypoints
    checkAxisAlignment() {
        // Ne pas faire cette vérification si on est en marche arrière forcée
        if (this.shouldReverseDirection) return;
        
        // Ne pas faire cette vérification pour le dernier waypoint
        if (this.currentTargetIndex >= this.path.length - 1) return;
        
        const pos = this.vehicle.obj.pos;
        // MODIFIÉ: Tenir compte de la nouvelle orientation (vers la gauche)
        const vehicleHeading = normalizeAngle(this.vehicle.obj.angle + 180);
        
        // Chercher le waypoint le plus loin qu'on pourrait directement atteindre
        // Mais pas plus d'un waypoint à la fois pour éviter les sauts trop grands
        const nextIndex = this.currentTargetIndex + 1;
        if (nextIndex < this.path.length) {
            const waypoint = this.path[nextIndex];
            const dist = distance(pos[0], pos[1], waypoint.x, waypoint.y);
            
            // Si le waypoint est à une distance raisonnable
            if (dist <= this.axisAlignmentDistance) {
                // Calculer l'angle vers le waypoint
                const waypointAngle = radToDeg(Math.atan2(
                    waypoint.y - pos[1],
                    waypoint.x - pos[0]
                ));
                
                // Vérifier si on est dans l'axe
                const angleDiff = Math.abs(normalizeAngle(vehicleHeading - waypointAngle));
                if (angleDiff <= this.axisAlignmentAngle) {
                    // On est dans l'axe, on peut passer au waypoint suivant
                    this.currentTargetIndex = nextIndex;
                    console.log(`${this.vehicle.name}: Saut au waypoint suivant (dans l'axe)`);
                    this.vehicle.resetMovementCounter(); // Réinitialiser le compteur de mouvement
                    this.waypointChangeTimer = 0; // Réinitialiser le timer
                }
            }
        }
    }

    // Trouver un point à une distance lookahead sur le chemin
    findLookaheadPoint() {
        const pos = this.vehicle.obj.pos;
        
        // Si on a dépassé le dernier waypoint, on utilise le dernier
        if (this.currentTargetIndex >= this.path.length) {
            return this.path[this.path.length - 1];
        }
        
        // Chercher à partir du point actuel
        let lookaheadIndex = this.currentTargetIndex;
        let found = false;
        
        // Chercher le premier point qui est à une distance suffisante
        for (let i = this.currentTargetIndex; i < this.path.length; i++) {
            const point = this.path[i];
            const dist = distance(pos[0], pos[1], point.x, point.y);
            
            // Si on est au dernier point ou qu'on a trouvé un point assez loin
            if (i === this.path.length - 1 || dist >= this.lookaheadDistance) {
                lookaheadIndex = i;
                found = true;
                break;
            }
        }
        
        // Si on n'a pas trouvé de point assez loin, utiliser le dernier
        if (!found && this.path.length > 0) {
            lookaheadIndex = this.path.length - 1;
        }
        
        return this.path[lookaheadIndex];
    }

    // Calculer l'angle de braquage pour atteindre le point cible
    // MODIFIÉ: Tenir compte de la nouvelle orientation (vers la gauche)
    calculateSteeringAngle(targetPoint) {
        const pos = this.vehicle.obj.pos;
        // MODIFIÉ: Ajouter 180 degrés à l'angle pour que l'avant soit vers la gauche
        const heading = degToRad(normalizeAngle(this.vehicle.obj.angle + 180));
        
        // Vecteur vers la cible dans le référentiel global
        const globalX = targetPoint.x - pos[0];
        const globalY = targetPoint.y - pos[1];
        
        // Conversion en référentiel local (relatif à la voiture)
        const localX = Math.cos(-heading) * globalX - Math.sin(-heading) * globalY;
        const localY = Math.sin(-heading) * globalX + Math.cos(-heading) * globalY;
        
        // Si on est très proche du point (pour éviter la division par zéro)
        if (Math.abs(localX) < 0.001 && Math.abs(localY) < 0.001) {
            return 0;
        }
        
        // Calculer la courbure (1/rayon) nécessaire pour atteindre le point
        const curvature = 2 * localY / (localX * localX + localY * localY);
        
        // Convertir en angle de braquage
        let steeringAngle = radToDeg(Math.atan(curvature * this.vehicle.wheelbase));
        
        // Si on recule, inverser l'angle de braquage
        const isPointInFront = this.isPointInFrontSector(targetPoint);
        const isMovingForward = !this.shouldReverseDirection && isPointInFront;
        const isMovingBackward = this.shouldReverseDirection || !isPointInFront;
        
        if ((localX < 0 && isMovingForward) || (localX > 0 && isMovingBackward)) {
            steeringAngle = -steeringAngle;
        }
        
        return Math.max(-this.vehicle.maxSteeringAngle, 
                      Math.min(this.vehicle.maxSteeringAngle, steeringAngle));
    }

    // Calculer la vitesse en fonction de la courbure et de la distance
    calculateVelocity(steeringAngle, targetPoint) {
        // Ralentir dans les virages serrés
        const steeringFactor = 1 - Math.abs(steeringAngle) / this.vehicle.maxSteeringAngle * 0.7;
        
        // Facteur de distance par rapport au dernier waypoint
        let distanceFactor = 1;
        
        // Si on est presque arrivé à destination, ralentir
        if (this.currentTargetIndex >= this.path.length - 2) {
            const pos = this.vehicle.obj.pos;
            const finalPoint = this.path[this.path.length - 1];
            const distToFinal = distance(pos[0], pos[1], finalPoint.x, finalPoint.y);
            
            if (distToFinal < this.slowdownDistance) {
                distanceFactor = Math.max(0.2, distToFinal / this.slowdownDistance);
            }
        }
        
        return this.vehicle.maxVelocity * steeringFactor * distanceFactor;
    }
}

/**
 * Planificateur de trajectoire avec waypoints prédéfinis
 */
class PathPlanner {
    constructor(startPos, goalPos, obstacles, bounds) {
        this.startPos = startPos;
        this.goalPos = goalPos;
        this.obstacles = obstacles || [];
        this.bounds = bounds || { 
            minX: 0, maxX: 1100, 
            minY: 0, maxY: 900 
        };
    }

    // Planifier un chemin de départ à arrivée
    planPath() {
        // Chemins prédéfinis pour les voitures (avec plus de waypoints pour un mouvement plus progressif)
        const predefinedPaths = {
            "golf": [
                { x: 515, y: 550, angle: 0 },
                { x: doorCenterX, y: 600, angle: 0 },  // Aligner avec la porte
                { x: doorCenterX, y: 500, angle: 0 },  // Entrée du garage
                { x: doorCenterX, y: 400, angle: 0 },  // Au milieu du garage
                { x: 600, y: 350, angle: -45 },       // Commencer à tourner
                { x: 650, y: 250, angle: -90 },       // Continuer à tourner
                { x: 672, y: 200, angle: -45 },       // Correction d'angle
                { x: 672, y: 162, angle: 0 }          // Position finale
            ],
            "porsche": [
                { x: 303, y: 344, angle: -90 },
                { x: 350, y: 500, angle: -45 },       // Avancer en tournant
                { x: doorCenterX, y: 600, angle: 0 },  // Aligner avec la porte
                { x: doorCenterX, y: 500, angle: 0 },  // Entrée du garage
                { x: doorCenterX, y: 400, angle: 0 },  // Au milieu du garage
                { x: 525, y: 375, angle: -45 },       // Commencer à tourner
                { x: 562, y: 350, angle: -90 },       // Continuer à tourner
                { x: 562, y: 332, angle: -90 }        // Position finale
            ]
        };
        
        // On retourne le chemin prédéfini correspondant au nom du véhicule
        const carName = this.startPos.name;
        if (predefinedPaths[carName]) {
            // Ajouter des angles de transition pour plus de fluidité
            return this.smoothPath(predefinedPaths[carName]);
        }
        
        // Chemin par défaut si aucun n'est trouvé
        return [
            { x: this.startPos.x, y: this.startPos.y, angle: this.startPos.angle },
            { x: this.goalPos.x, y: this.goalPos.y, angle: this.goalPos.angle }
        ];
    }
    
    // Lisser le chemin en ajoutant des points intermédiaires
    smoothPath(path) {
        if (path.length < 2) return path;
        
        const smoothed = [];
        
        for (let i = 0; i < path.length - 1; i++) {
            const current = path[i];
            const next = path[i + 1];
            
            smoothed.push(current);
            
            // Calculer la distance
            const d = distance(current.x, current.y, next.x, next.y);
            
            // Si la distance est grande, ajouter des points intermédiaires
            if (d > 100) {
                const numPoints = Math.floor(d / 50);
                for (let j = 1; j < numPoints; j++) {
                    const t = j / numPoints;
                    const x = lerp(current.x, next.x, t);
                    const y = lerp(current.y, next.y, t);
                    
                    // Interpoler aussi l'angle
                    const angleDiff = shortestAngle(current.angle, next.angle);
                    const angle = normalizeAngle(current.angle + angleDiff * t);
                    
                    smoothed.push({ x, y, angle });
                }
            }
        }
        
        // Ajouter le dernier point
        smoothed.push(path[path.length - 1]);
        
        return smoothed;
    }
}

// Exporter les classes pour utilisation dans autopilot.js
window.VehicleModel = VehicleModel;
window.PurePursuitController = PurePursuitController;
window.PathPlanner = PathPlanner;
