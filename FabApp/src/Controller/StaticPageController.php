<?php

namespace App\Controller;

use App\Service\MarkdownDocService;
use App\Service\SiteSettingService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

final class StaticPageController extends AbstractController
{
    #[Route('/documentation', name: 'app_documentation', methods: ['GET'])]
    public function documentation(): Response
    {
        return $this->render('site/static/documentation.html.twig');
    }

    #[Route('/support', name: 'app_support', methods: ['GET'])]
    public function support(): Response
    {
        return $this->render('site/static/support.html.twig');
    }

    #[Route('/api-docs', name: 'app_api_docs', methods: ['GET'])]
    public function apiDocs(): Response
    {
        return $this->render('site/static/api-docs.html.twig');
    }

    #[Route('/statut', name: 'app_status', methods: ['GET'])]
    public function status(): Response
    {
        return $this->render('site/static/status.html.twig');
    }

    /**
     * The build roadmap and the project handover notes, rendered from the
     * markdown files in `docs/`.
     *
     * Reading the files rather than restating them in Twig is the whole point:
     * the previous version of this page was a hand-written copy of the plan and
     * went stale within one work session. Now the page cannot disagree with the
     * repository.
     */
    #[Route('/roadmap', name: 'app_roadmap', methods: ['GET'])]
    public function roadmap(MarkdownDocService $docs): Response
    {
        return $this->render('site/static/roadmap.html.twig', [
            'roadmapHtml' => $docs->render('roadmap'),
            'stateHtml' => $docs->render('state'),
            'roadmapUpdatedAt' => $docs->updatedAt('roadmap'),
        ]);
    }

    #[Route('/reglement', name: 'app_lab_rules', methods: ['GET'])]
    public function labRules(SiteSettingService $siteSettings): Response
    {
        return $this->render('site/static/lab-rules.html.twig', [
            'labRulesHtml' => $siteSettings->getLabRulesHtml(),
            'labRulesPdfUrl' => $siteSettings->getLabRulesPdfUrl(),
        ]);
    }

    #[Route('/mentions-legales', name: 'app_legal_notice', methods: ['GET'])]
    public function legalNotice(): Response
    {
        return $this->render('site/static/legal-notice.html.twig');
    }

    #[Route('/confidentialite', name: 'app_privacy_policy', methods: ['GET'])]
    public function privacyPolicy(): Response
    {
        return $this->render('site/static/privacy-policy.html.twig');
    }

    #[Route('/conditions-utilisation', name: 'app_terms', methods: ['GET'])]
    public function terms(): Response
    {
        return $this->render('site/static/terms.html.twig');
    }
}
