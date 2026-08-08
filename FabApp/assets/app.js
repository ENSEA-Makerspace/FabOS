/*
 * Point d'entrée AssetMapper — chargé par `importmap('app')`.
 *
 * ⚠️ Ce fichier ne charge QUE Stimulus. Turbo est délibérément désactivé
 * (voir `assets/controllers.json`) : il intercepterait toute la navigation du
 * site, et les 21 blocs `<script>` inline des templates ne sont pas prêts pour
 * ça (certains s'accrochent à `DOMContentLoaded`, qui ne se déclenche plus sur
 * une visite Turbo ; d'autres posent des listeners sur `document` sans jamais
 * les retirer, donc ils s'empileraient). Rallumer Turbo est une session à part.
 *
 * ⚠️ N'importe PAS `./styles/app.css` ici. Ce fichier était le scaffolding par
 * défaut de Symfony (`body { background-color: skyblue }`) et il est resté dans
 * le dépôt pendant des mois sans jamais être chargé. Le CSS du site vit dans
 * `public/css/`, servi par `asset()` avec un cache-buster `?v=` — pas par
 * AssetMapper. Mélanger les deux ferait qu'une feuille échapperait au buster.
 */
import { application } from './stimulus_bootstrap.js';
import AdminListFilterController from './controllers/admin_list_filter_controller.js';

application.register('admin-list-filter', AdminListFilterController);
